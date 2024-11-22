#include <ur3e_mrc/ur3e_mrc_robotiq.hpp>

// the code is based on sdurobotics ur_rtde gripper example

namespace grip_group_ur3e
{
class UR3eMRCRobotiq
{
public:
  explicit UR3eMRCRobotiq(rclcpp::Node::SharedPtr node) : node_(node), gripper_("192.168.77.22", 63352, true)
  {
    comm_cb_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // gripper = ur_rtde::RobotiqGripper("192.168.77.22", 63352, true);
    gripper_.connect();
    RCLCPP_INFO(node_->get_logger(), "Activating gripper");
    gripper_.activate();

    // // Test setting of position units and conversion of position values
    // gripper.setUnit(ur_rtde::RobotiqGripper::POSITION, ur_rtde::RobotiqGripper::UNIT_DEVICE);
    // RCLCPP_INFO(node_->get_logger(), "OpenPosition: %f  ClosedPosition: %f", gripper.getOpenPosition(), gripper.getClosedPosition());
    // int status = gripper.move(0, 1, 0, ur_rtde::RobotiqGripper::WAIT_FINISHED);    
    // RCLCPP_INFO(node_->get_logger(), "The gripper's status is: %i", status);

    // Autocalibrating the gripper
    RCLCPP_INFO(node_->get_logger(), "Autocalibrating the gripper");
    gripper_.autoCalibrate();

    // Set percent position units
    gripper_.setUnit(ur_rtde::RobotiqGripper::POSITION, ur_rtde::RobotiqGripper::UNIT_PERCENT); //UNIT_NORMALIZED
    RCLCPP_INFO(node_->get_logger(), "OpenPosition: %f  ClosedPosition: %f", gripper_.getOpenPosition(), gripper_.getClosedPosition());    

    // bool grip_isClosed = gripper_.isClosed();
    // if (grip_isClosed) {RCLCPP_INFO(node_->get_logger(), "gripper is closed");}
    // else if (!grip_isClosed) {RCLCPP_INFO(node_->get_logger(), "gripper not closed");}
    // bool grip_isOpen = gripper_.isOpen();
    // if (grip_isOpen) {RCLCPP_INFO(node_->get_logger(), "gripper is open");}
    // else if (!grip_isOpen) {RCLCPP_INFO(node_->get_logger(), "gripper not open");}


    RCLCPP_INFO(node_->get_logger(), "Closing the gripper");
    gripper_.move(0, 1, 0, ur_rtde::RobotiqGripper::WAIT_FINISHED);
    rclcpp::sleep_for(std::chrono::seconds(2));

    RCLCPP_INFO(node_->get_logger(), "Opening the gripper");
    gripper_.move(100, 1, 0, ur_rtde::RobotiqGripper::WAIT_FINISHED);

    // preset force and and speed
    gripper_.setForce(0.0);
    gripper_.setSpeed(0.4);

    rclcpp::SubscriptionOptions options_comm;
    options_comm.callback_group = comm_cb_group_;
    sub_comm_ = node_->create_subscription<std_msgs::msg::Bool>("ur3/grip_command", 10, std::bind(&UR3eMRCRobotiq::comm_callback, this, std::placeholders::_1), options_comm);
    RCLCPP_INFO(node_->get_logger(), "Subscribed to ur3/grip_command");

  }

  void gripperDisconnect()
  {
    RCLCPP_INFO(node_->get_logger(), "Disconnecting gripper");
    gripper_.disconnect();
  }

private:
  rclcpp::Node::SharedPtr node_;
  ur_rtde::RobotiqGripper gripper_;

  rclcpp::CallbackGroup::SharedPtr comm_cb_group_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_comm_;  

  void comm_callback(const std_msgs::msg::Bool & msg)
  {

    if (msg.data && !gripper_.isOpen())
    {
      RCLCPP_INFO(node_->get_logger(), "Opening the gripper");
      gripper_.open(-1, -1, ur_rtde::RobotiqGripper::WAIT_FINISHED); //opening the gripper if command is true
    }
    else if (!msg.data && !gripper_.isClosed())
    {
      RCLCPP_INFO(node_->get_logger(), "Closing the gripper");
      gripper_.close(-1, -1, ur_rtde::RobotiqGripper::WAIT_FINISHED); //closing the gripper if command is false
    }

    auto grip_pos = gripper_.getCurrentPosition();
    RCLCPP_INFO(node_->get_logger(), "gripper current position is: %f", grip_pos);    
  }

};
}   // namespace grip_group_ur3e

int main(int argc, char * argv[])
{

  rclcpp::init(argc, argv);
  auto robotiq_node = std::make_shared<rclcpp::Node>("ur3e_mrc_robotiq");
  rclcpp::executors::MultiThreadedExecutor executor;
  
  RCLCPP_INFO(robotiq_node->get_logger(), "Starting the Hand-E control node");
  grip_group_ur3e::UR3eMRCRobotiq ur3e_grip(robotiq_node);
  
  executor.add_node(robotiq_node);  
  executor.spin();

  // shutting down procedures
  RCLCPP_INFO(robotiq_node->get_logger(), "Keyboard interrupt, shutting down");
  ur3e_grip.gripperDisconnect();

  rclcpp::shutdown();

  return 0;
}