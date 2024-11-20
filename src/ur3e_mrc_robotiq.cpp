#include <ur3e_mrc/ur3e_mrc_robotiq.hpp>

// the code is based on sdurobotics ur_rtde gripper example

namespace grip_group_ur3e
{
class UR3eMRCRobotiq
{
public:
  explicit UR3eMRCRobotiq(rclcpp::Node::SharedPtr node) : node_(node)
  {
    ur_rtde::RobotiqGripper gripper("127.0.0.1", 63352, true);
    gripper.connect();
    RCLCPP_INFO(node_->get_logger(), "Activating gripper");

    int status = gripper.move(1, 1, 0, ur_rtde::RobotiqGripper::WAIT_FINISHED);
    RCLCPP_INFO(node_->get_logger(), "The gripper's status is: %i", status);

  }

private:
  rclcpp::Node::SharedPtr node_;

};
}   // namespace grip_group_ur3e

int main(int argc, char * argv[])
{

  rclcpp::init(argc, argv);
  auto robotiq_node = std::make_shared<rclcpp::Node>("ur3e_mrc_robotiq");
  rclcpp::executors::MultiThreadedExecutor executor;
  
  RCLCPP_INFO(robotiq_node->get_logger(), "Starting the Hand-E control node");
  grip_group_ur3e::UR3eMRCRobotiq ur3e_ctrl(robotiq_node);
  
  executor.add_node(robotiq_node);  
  executor.spin();

  // shutting down procedures
  RCLCPP_INFO(robotiq_node->get_logger(), "Keyboard interrupt, shutting down");

  rclcpp::shutdown();

  return 0;
}