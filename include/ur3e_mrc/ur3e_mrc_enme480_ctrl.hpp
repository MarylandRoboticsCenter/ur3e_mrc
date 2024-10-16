#include <chrono>
#include <memory>
#include <functional>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_action/create_client.hpp"

#include "boost/math/constants/constants.hpp"

#include "sensor_msgs/msg/joint_state.hpp"

#include "joint_trajectory_controller/joint_trajectory_controller.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

#include "controller_manager_msgs/srv/list_controllers.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"

#include "ur_msgs/msg/io_states.hpp"
#include "ur_msgs/srv/set_io.hpp"

#include "ur3e_mrc/msg/command_ur3e.hpp"



// #include "ur_robot_driver/msg/hardware_interface.hpp"

// #include <signal.h>
// #include <ros/xmlrpc_manager.h>

// #include <std_msgs/String.h>
// #include <sensor_msgs/JointState.h>


// typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> TrajectoryClient;

// class UR3eArm
// {
// private:
//     ros::NodeHandle nh_;
//     TrajectoryClient *trajectory_client_;
//     ros::Subscriber sub_comm_;

//     ros::ServiceClient ctrl_manager_srv_;
//     ros::ServiceClient ctrl_list_srv_;
//     ros::ServiceClient io_client_srv_;

//     void ctrlRunCheck();

//     bool init_status_ = false;

// public:
//     UR3eArm();
//     ~UR3eArm();
//     void commCallback(const ur3e_mrc::command &msg);

//     void initGoal(control_msgs::FollowJointTrajectoryGoal &goal);
//     void sendTrajectory(control_msgs::FollowJointTrajectoryGoal goal);

//     void ioCtrl(unsigned short int numPin, unsigned short int valPin);

//     // control_msgs::FollowJointTrajectoryGoal ur3eTrajectory(); //const ur3e_mrc::position &msg
//     bool initStatus();
//     actionlib::SimpleClientGoalState getState();

// protected:
// };