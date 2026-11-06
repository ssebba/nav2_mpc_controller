#ifndef MPC_BENCHMARK__GROUND_TRUTH_RELAY_HPP_
#define MPC_BENCHMARK__GROUND_TRUTH_RELAY_HPP_

#include <gz/msgs/pose_v.pb.h>

#include <gz/transport/Node.hh>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

#include "mpc_benchmark/ground_truth_relay_parameters.hpp"

namespace mpc_benchmark
{

/**
 * @brief Republishes the pose of one Gazebo model as geometry_msgs/PoseStamped (~/pose).
 *
 * The benchmark ground truth: wheel odometry drifts by tens of centimeters when the wheels
 * slip in tight swerves. The pose is read from the simulator's pose topic through
 * gz-transport because ros_gz_bridge drops the entity names when it converts gz.msgs.Pose_V,
 * so the robot could not be told apart from the other models.
 */
class GroundTruthRelay : public rclcpp::Node
{
public:
  explicit GroundTruthRelay(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void onPoses(const gz::msgs::Pose_V & msg);

  std::shared_ptr<ground_truth_relay::ParamListener> param_listener_;
  ground_truth_relay::Params params_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
  gz::transport::Node gz_node_;
};

}  // namespace mpc_benchmark

#endif  // MPC_BENCHMARK__GROUND_TRUTH_RELAY_HPP_
