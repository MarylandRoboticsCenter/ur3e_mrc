#include <ur3e_mrc/ur3e_mrc_enme480_topics.hpp>


class UR3eENME480JS : public rclcpp::Node
{
public:
  UR3eENME480JS() : Node("ur3e_mrc_enme480_js")
  {
    sub_js_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10, std::bind(&UR3eENME480JS::js_callback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Subscribed to joint states");

    pos_pub_ = this->create_publisher<ur3e_mrc::msg::PositionUR3e>("ur3/position", 10);
  }

private:
  void js_callback(const sensor_msgs::msg::JointState & msg) const
  {
    ur3e_mrc::msg::PositionUR3e pos_msg;

    pos_msg.position = {msg.position[2] + boost::math::constants::pi<double>() / 2, msg.position[1], msg.position[0], msg.position[3], msg.position[4], msg.position[5]};
    pos_msg.is_ready = true;

    pos_pub_->publish(pos_msg);
  }

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_js_;
  rclcpp::Publisher<ur3e_mrc::msg::PositionUR3e>::SharedPtr pos_pub_;
};

class UR3eENME480IO : public rclcpp::Node
{
public:
  UR3eENME480IO() : Node("ur3e_mrc_enme480_io")
  {
    sub_io_ = this->create_subscription<ur_msgs::msg::IOStates>(
      "/io_and_status_controller/io_states", 10, std::bind(&UR3eENME480IO::io_callback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Subscribed to hardware io states");
    

    grip_grasp_pub_ = this->create_publisher<std_msgs::msg::Bool>("gripper/grasping", 10);
    grip_inp_pub_ = this->create_publisher<ur3e_mrc::msg::GripperInput>("ur3/gripper_input", 10);

  }

private:
  void io_callback(const ur_msgs::msg::IOStates & msg) const
  { 
    std_msgs::msg::Bool grip_grasp_msg;
    ur3e_mrc::msg::GripperInput grip_inp_msg;

    grip_grasp_msg.data = msg.digital_in_states[0].state;

    if (msg.digital_in_states[0].state)
      grip_inp_msg.dig_in = 1;
    else
      grip_inp_msg.dig_in = 0;

    grip_inp_msg.a_in0 = msg.analog_in_states[0].state;
    grip_inp_msg.a_in1 = 0.0;

    grip_grasp_pub_->publish(grip_grasp_msg);
    grip_inp_pub_->publish(grip_inp_msg);
    
    // RCLCPP_INFO(this->get_logger(), "I got js message");
  }

  rclcpp::Subscription<ur_msgs::msg::IOStates>::SharedPtr sub_io_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr grip_grasp_pub_;
  rclcpp::Publisher<ur3e_mrc::msg::GripperInput>::SharedPtr grip_inp_pub_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor executor;
  auto jsnode = std::make_shared<UR3eENME480JS>();
  auto ionode = std::make_shared<UR3eENME480IO>();
  executor.add_node(jsnode);
  executor.add_node(ionode);
  executor.spin();

  // rclcpp::spin(std::make_shared<UR3eENME480Topics>());
  rclcpp::shutdown();

  return 0;
}