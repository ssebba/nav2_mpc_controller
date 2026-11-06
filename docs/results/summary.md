### S1 - S-curve tracking (controller only)

| Controller | Success | Time [s] | Cross-track mean [cm] | Cross-track max [cm] | Compute p50 [ms] | Compute p95 [ms] | RMS ang. accel [rad/s²] |
|---|---|---|---|---|---|---|---|
| MPC-RTI (ours) | 5/5 | 26.1 ± 0.1 | 0.65 ± 0.00 | 1.23 ± 0.01 | 0.26 | 0.38 | 0.20 ± 0.00 |
| MPC-LTV (ours) | 5/5 | 26.1 ± 0.0 | 0.64 ± 0.00 | 1.23 ± 0.02 | 0.29 | 0.44 | 0.20 ± 0.00 |
| Kanayama (ours) | 5/5 | 25.7 ± 0.0 | 1.48 ± 0.01 | 2.69 ± 0.01 | 0.04 | 0.07 | 0.34 ± 0.00 |
| MPPI | 5/5 | 23.8 ± 0.0 | 11.12 ± 0.06 | 21.68 ± 0.17 | 3.72 | 7.36 | 0.43 ± 0.00 |
| RPP | 5/5 | 27.7 ± 0.1 | 3.86 ± 0.05 | 6.68 ± 0.12 | 0.05 | 0.08 | 0.17 ± 0.00 |
| DWB | 5/5 | 26.3 ± 0.1 | 3.70 ± 0.10 | 6.61 ± 0.16 | 5.61 | 7.39 | 1.10 ± 0.08 |

### S2 - waypoints through the pillar grid

| Controller | Success | Time [s] | Min clearance [m] | Runs with contact | Compute p50 [ms] | Compute p95 [ms] | RMS ang. accel [rad/s²] |
|---|---|---|---|---|---|---|---|
| MPC-RTI (ours) | 5/5 | 54.5 ± 1.3 | 0.07 | 0 | 0.37 | 0.57 | 1.10 ± 0.05 |
| MPC-LTV (ours) | 5/5 | 54.5 ± 0.8 | 0.09 | 0 | 0.38 | 0.61 | 1.20 ± 0.05 |
| Kanayama (ours) | 3/5 | 54.3 ± 3.8 | 0.05 | 0 | 0.04 | 0.07 | 1.72 ± 0.05 |
| MPPI | 5/5 | 58.7 ± 2.6 | 0.01 | 0 | 3.02 | 6.07 | 0.28 ± 0.03 |
| RPP | 5/5 | 68.2 ± 13.6 | -0.02 | 2 | 0.05 | 0.09 | 1.01 ± 0.75 |
| DWB | 5/5 | 52.5 ± 1.7 | 0.05 | 0 | 5.90 | 7.29 | 1.25 ± 0.15 |

### S3 - obstacles not in the map, avoided by the controller alone

| Controller | Success | Time [s] | Min clearance [m] | Runs with contact | Compute p50 [ms] | Compute p95 [ms] | RMS ang. accel [rad/s²] |
|---|---|---|---|---|---|---|---|
| MPC-RTI (ours) | 5/5 | 29.5 ± 2.0 | 0.02 | 0 | 0.38 | 3.48 | 1.46 ± 0.17 |
| MPC-LTV (ours) | 5/5 | 38.3 ± 13.3 | 0.00 | 0 | 0.33 | 3.87 | 1.68 ± 0.15 |
| Kanayama (ours) | 0/5 | n/a | 0.21 | 0 | 0.03 | 0.06 | n/a |
| MPPI | 0/5 | n/a | 0.23 | 0 | 3.29 | 4.25 | n/a |
| RPP | 0/5 | n/a | 0.16 | 0 | 0.05 | 0.09 | n/a |
| DWB | 0/5 | n/a | 0.05 | 0 | 6.68 | 8.98 | n/a |
