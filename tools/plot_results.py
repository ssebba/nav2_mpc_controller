#!/usr/bin/env python3
"""Offline analysis of a benchmark campaign (output of run_benchmark.sh).

  tools/plot_results.py <results_dir> [--out docs/results] [--timing solver_timing.csv]

Writes summary.md (tables for the README), runs.csv (metrics of every run) and the figures
used in the README.
This is the only Python in the repository besides launch files: analysis, not runtime.
"""

import argparse
import glob
import os

import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402

CONTROLLER_ORDER = ['mpc_rti', 'mpc_ltv', 'baseline', 'mppi', 'rpp', 'dwb']
LABELS = {
    'mpc_rti': 'MPC-RTI (ours)', 'mpc_ltv': 'MPC-LTV (ours)', 'baseline': 'Kanayama (ours)',
    'mppi': 'MPPI', 'rpp': 'RPP', 'dwb': 'DWB',
}
COLORS = {
    'mpc_rti': '#1f6feb', 'mpc_ltv': '#58a6ff', 'baseline': '#8b949e',
    'mppi': '#d29922', 'rpp': '#3fb950', 'dwb': '#db6d28',
}
SCENARIO_TITLES = {
    's1_tracking': 'S1 - S-curve tracking (controller only)',
    's2_waypoints': 'S2 - waypoints through the pillar grid',
    's3_unknown_obstacles': 'S3 - obstacles not in the map, avoided by the controller alone',
}


def load(results_dir):
    runs = pd.read_csv(os.path.join(results_dir, 'results.csv'))
    summaries = [pd.read_csv(f) for f in glob.glob(os.path.join(results_dir, '*_summary.csv'))]
    summary = pd.concat(summaries) if summaries else pd.DataFrame(columns=['run_id'])
    # The runner's duration (goal sent -> result) is the task time; keep the recorder's apart.
    summary = summary.rename(columns={'duration_s': 'recorded_duration_s'})
    df = runs.merge(summary, on='run_id', how='left')
    df = df.drop_duplicates(subset='run_id', keep='last')
    present = [c for c in CONTROLLER_ORDER if c in set(df['controller'])]
    return df, present


def compute_times(results_dir, run_ids):
    values = []
    for run_id in run_ids:
        path = os.path.join(results_dir, f'{run_id}_compute.csv')
        if os.path.exists(path):
            values.extend(pd.read_csv(path)['compute_time_ms'].tolist())
    return np.array(values)


def fmt(mean, std, digits):
    if np.isnan(mean):
        return 'n/a'
    return f'{mean:.{digits}f} ± {std:.{digits}f}' if not np.isnan(std) else f'{mean:.{digits}f}'


def summary_tables(df, controllers, results_dir):
    lines = []
    for scenario in [s for s in SCENARIO_TITLES if s in set(df['scenario'])]:
        sdf = df[df['scenario'] == scenario]
        lines += [f'### {SCENARIO_TITLES[scenario]}', '']
        header = ['Controller', 'Success', 'Time [s]']
        if scenario == 's1_tracking':
            header += ['Cross-track mean [cm]', 'Cross-track max [cm]']
        else:
            header += ['Min clearance [m]', 'Runs with contact']
        header += ['Compute p50 [ms]', 'Compute p95 [ms]', 'RMS ang. accel [rad/s²]']
        lines += ['| ' + ' | '.join(header) + ' |', '|' + '---|' * len(header)]
        for c in controllers:
            cdf = sdf[sdf['controller'] == c]
            if cdf.empty:
                continue
            ok = cdf[cdf['success'] == 1]
            times = compute_times(results_dir, cdf['run_id'])
            row = [LABELS[c], f'{len(ok)}/{len(cdf)}', fmt(ok['duration_s'].mean(), ok['duration_s'].std(), 1)]
            if scenario == 's1_tracking':
                row += [fmt(100 * ok['cte_mean_m'].mean(), 100 * ok['cte_mean_m'].std(), 2),
                        fmt(100 * ok['cte_max_m'].mean(), 100 * ok['cte_max_m'].std(), 2)]
            else:
                row += [fmt(cdf['clearance_min_m'].min(), np.nan, 2),
                        str(int((cdf['collision_samples'] > 0).sum()))]
            row += [f'{np.percentile(times, 50):.2f}' if times.size else 'n/a',
                    f'{np.percentile(times, 95):.2f}' if times.size else 'n/a',
                    fmt(ok['rms_angular_accel'].mean(), ok['rms_angular_accel'].std(), 2)]
            lines.append('| ' + ' | '.join(row) + ' |')
        lines.append('')
    return '\n'.join(lines)


def plot_compute_times(df, controllers, results_dir, out):
    data, labels, colors = [], [], []
    for c in controllers:
        times = compute_times(results_dir, df[df['controller'] == c]['run_id'])
        if times.size:
            data.append(times)
            labels.append(LABELS[c])
            colors.append(COLORS[c])
    if not data:
        return
    fig, ax = plt.subplots(figsize=(8, 3.6))
    box = ax.boxplot(data, vert=False, showfliers=False, patch_artist=True, widths=0.6)
    for patch, color in zip(box['boxes'], colors):
        patch.set_facecolor(color)
        patch.set_alpha(0.75)
    ax.set_yticks(range(1, len(labels) + 1), labels)
    ax.set_xscale('log')
    ax.set_xlabel('computeVelocityCommands() wall time per cycle [ms] (log scale)')
    ax.axvline(50.0, color='#cf222e', linestyle='--', linewidth=1)
    ax.text(50.0, len(labels) + 0.35, ' 20 Hz budget', color='#cf222e', fontsize=8)
    ax.grid(axis='x', which='both', alpha=0.3)
    ax.set_title('Controller compute time, all scenarios')
    fig.tight_layout()
    fig.savefig(os.path.join(out, 'compute_time.png'), dpi=150, bbox_inches='tight')
    plt.close(fig)


def s_curve_reference(scenarios_file):
    """S1 reference path, rebuilt like mpc_benchmark::makeReferencePath (start along +x)."""
    import yaml
    with open(scenarios_file, encoding='utf-8') as f:
        p = yaml.safe_load(f)['s1_tracking']['runner']['path']
    x = np.linspace(0.0, p['length'], 600)
    y = p['size'] * np.sin(2.0 * np.pi * x / p['wavelength'])
    yaw0 = np.arctan2(y[1] - y[0], x[1] - x[0])
    c, s = np.cos(-yaw0), np.sin(-yaw0)
    return c * x - s * y, s * x + c * y


def plot_tracking(df, controllers, results_dir, out, scenarios_file=None):
    sdf = df[(df['scenario'] == 's1_tracking') & (df['success'] == 1)]
    if sdf.empty:
        return
    fig, (ax_xy, ax_cte) = plt.subplots(1, 2, figsize=(11, 4.2), gridspec_kw={'width_ratios': [1.25, 1]})
    if scenarios_file and os.path.exists(scenarios_file):
        rx, ry = s_curve_reference(scenarios_file)
        ax_xy.plot(rx, ry, color='black', linestyle='--', linewidth=1.0, label='reference')
    reference_drawn = False
    for c in controllers:
        runs = sdf[sdf['controller'] == c]
        if runs.empty:
            continue
        run_id = runs.iloc[0]['run_id']
        samples = pd.read_csv(os.path.join(results_dir, f'{run_id}_samples.csv'))
        ax_xy.plot(samples['x'], samples['y'], color=COLORS[c], linewidth=1.4, label=LABELS[c])
        ax_cte.plot(samples['t'], 100 * samples['cross_track'], color=COLORS[c], linewidth=1.0,
                    label=LABELS[c])
        reference_drawn = True
    if reference_drawn:
        ax_xy.set_aspect('equal')
        ax_xy.set_xlabel('x [m]')
        ax_xy.set_ylabel('y [m]')
        ax_xy.set_title('S1: executed trajectories (first run)')
        ax_xy.grid(alpha=0.3)
        ax_xy.legend(fontsize=8, loc='best')
        ax_cte.set_xlabel('time [s]')
        ax_cte.set_ylabel('cross-track error [cm]')
        ax_cte.set_title('S1: distance from the reference')
        ax_cte.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(out, 'tracking.png'), dpi=150, bbox_inches='tight')
    plt.close(fig)


S3_BOXES = [(1.5, 0.08), (3.0, -0.08), (4.5, 0.08)]  # centers of the 0.3 m boxes


def plot_obstacle_runs(df, controllers, results_dir, out):
    sdf = df[df['scenario'] == 's3_unknown_obstacles']
    if sdf.empty:
        return
    fig, ax = plt.subplots(figsize=(10, 3.6))
    ax.plot([0.0, 6.0], [0.0, 0.0], color='black', linestyle='--', linewidth=1,
            label='initial global plan (blind to the boxes)')
    for bx, by in S3_BOXES:
        ax.add_patch(plt.Rectangle((bx - 0.15, by - 0.15), 0.3, 0.3, color='#cf222e', alpha=0.8))
    for c in controllers:
        runs = sdf[sdf['controller'] == c]
        if runs.empty:
            continue
        ok = runs[runs['success'] == 1]
        failed = runs[runs['success'] != 1]
        # Every run: lines for the successful ones, a cross where the others stopped.
        for i, run_id in enumerate(ok['run_id']):
            samples = pd.read_csv(os.path.join(results_dir, f'{run_id}_samples.csv'))
            ax.plot(samples['x'], samples['y'], color=COLORS[c], linewidth=1.1, alpha=0.85,
                    label=f'{LABELS[c]}: {len(ok)}/{len(runs)} reached the goal' if i == 0 else None)
        for i, run_id in enumerate(failed['run_id']):
            samples = pd.read_csv(os.path.join(results_dir, f'{run_id}_samples.csv'))
            ax.plot(samples['x'].iloc[-1], samples['y'].iloc[-1], marker='x', markersize=7,
                    color=COLORS[c], linestyle='none',
                    label=f'{LABELS[c]}: stopped ({len(failed)}/{len(runs)})' if i == 0 else None)
    ax.set_aspect('equal')
    ax.set_xlim(-0.3, 6.3)
    ax.set_ylim(-0.9, 0.6)
    ax.set_xlabel('x [m] (simulator ground truth)')
    ax.set_ylabel('y [m]')
    ax.set_title('S3: three boxes (red) unknown to the planner, every run of every controller')
    ax.legend(fontsize=7, loc='upper center', ncol=3, bbox_to_anchor=(0.5, -0.28))
    fig.savefig(os.path.join(out, 'obstacles.png'), dpi=150, bbox_inches='tight')
    plt.close(fig)


def plot_solver_timing(timing_csv, out):
    if not timing_csv or not os.path.exists(timing_csv):
        return
    t = pd.read_csv(timing_csv)
    t = t[(t['scenario'] == 'obstacles') & (t['solver'] != 'baseline')]
    fig, ax = plt.subplots(figsize=(6.5, 3.6))
    for solver, color in [('ltv_mpc', '#58a6ff'), ('nmpc_rti', '#1f6feb'), ('nmpc_sqp3', '#8250df')]:
        st = t[t['solver'] == solver]
        ax.plot(st['horizon'], st['p50_ms'], 'o-', color=color, label=f'{solver} (median)')
        ax.fill_between(st['horizon'], st['p50_ms'], st['p95_ms'], color=color, alpha=0.15)
    ax.set_xlabel('prediction horizon N (dt = 0.1 s)')
    ax.set_ylabel('solve time [ms]')
    ax.set_title('OSQP solve time vs horizon (band: median to p95)')
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(out, 'solver_timing.png'), dpi=150, bbox_inches='tight')
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument('results_dir')
    parser.add_argument('--out', default='docs/results')
    parser.add_argument('--timing', default='', help='CSV produced by mpc_core_timing')
    parser.add_argument('--scenarios', default=os.path.join(
        os.path.dirname(os.path.abspath(__file__)), '..', 'mpc_benchmark', 'config', 'scenarios.yaml'))
    args = parser.parse_args()
    os.makedirs(args.out, exist_ok=True)

    df, controllers = load(args.results_dir)
    # Per-run metrics, so that the tables can be checked without the raw samples.
    df.to_csv(os.path.join(args.out, 'runs.csv'), index=False, float_format='%.5g')
    with open(os.path.join(args.out, 'summary.md'), 'w', encoding='utf-8') as f:
        f.write(summary_tables(df, controllers, args.results_dir))
    plot_compute_times(df, controllers, args.results_dir, args.out)
    plot_tracking(df, controllers, args.results_dir, args.out, args.scenarios)
    plot_obstacle_runs(df, controllers, args.results_dir, args.out)
    plot_solver_timing(args.timing, args.out)
    print(f'{len(df)} runs, controllers: {controllers}; outputs in {args.out}')


if __name__ == '__main__':
    main()
