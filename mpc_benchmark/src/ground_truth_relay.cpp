#include "mpc_benchmark/ground_truth_relay.hpp"

#include <stdexcept>

#include "rclcpp_components/register_node_macro.hpp"

namespace mpc_benchmark
{

GroundTruthRelay::GroundTruthRelay(const rclcpp::NodeOptions & options)
: rclcpp::Node("ground_truth_relay", options),
  param_listener_(
    std::make_shared<ground_truth_relay::ParamListener>(get_node_parameters_interface())),
  params_(param_listener_->get_params())
{
  publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>("~/pose", 10);
  if (!gz_node_.Subscribe(params_.gz_topic, &GroundTruthRelay::onPoses, this)) {
    throw std::runtime_error("GroundTruthRelay: cannot subscribe to " + params_.gz_topic);
  }
  RCLCPP_INFO(
    get_logger(), "Relaying the pose of '%s' from %s", params_.model.c_str(),
    params_.gz_topic.c_str());
}

void GroundTruthRelay::onPoses(const gz::msgs::Pose_V & msg)
{
  for (const auto & pose : msg.pose()) {
    if (pose.name() != params_.model) {
      continue;
    }
    geometry_msgs::msg::PoseStamped out;
    out.header.stamp.sec = static_cast<int32_t>(msg.header().stamp().sec());
    out.header.stamp.nanosec = static_cast<uint32_t>(msg.header().stamp().nsec());
    out.header.frame_id = params_.frame_id;
    out.pose.position.x = pose.position().x();
    out.pose.position.y = pose.position().y();
    out.pose.position.z = pose.position().z();
    out.pose.orientation.x = pose.orientation().x();
    out.pose.orientation.y = pose.orientation().y();
    out.pose.orientation.z = pose.orientation().z();
    out.pose.orientation.w = pose.orientation().w();
    publisher_->publish(out);
    return;
  }
}

}  // namespace mpc_benchmark

RCLCPP_COMPONENTS_REGISTER_NODE(mpc_benchmark::GroundTruthRelay)
