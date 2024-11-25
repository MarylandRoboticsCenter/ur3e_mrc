cmake_minimum_required(VERSION 3.8)
project(ur3e_mrc)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(controller_manager REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(joint_trajectory_controller REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(ur_msgs REQUIRED)
find_package(ur_robot_driver REQUIRED)
find_package(ur_rtde REQUIRED)

find_package(Boost REQUIRED)
find_package(rosidl_default_generators REQUIRED)

include_directories(include ${Boost_INCLUDE_DIRS}s)

set(msg_files
  msg/CommandUR3e.msg
  msg/GripperInput.msg
  msg/PositionUR3e.msg
)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  "srv/GripperCommand.srv"
)
ament_export_dependencies(rosidl_default_runtime)

add_executable(ur3e_mrc_enme480_topics src/ur3e_mrc_enme480_topics.cpp)
ament_target_dependencies(ur3e_mrc_enme480_topics rclcpp std_msgs sensor_msgs ur_msgs)

add_executable(ur3e_mrc_enme480_ctrl src/ur3e_mrc_enme480_ctrl.cpp)
ament_target_dependencies(ur3e_mrc_enme480_ctrl 
  controller_manager
  controller_manager_msgs
  control_msgs
  joint_trajectory_controller
  rclcpp 
  rclcpp_action
  sensor_msgs
  std_msgs 
  trajectory_msgs
  ur_msgs
)

add_executable(ur3e_mrc_robotiq src/ur3e_mrc_robotiq.cpp)
ament_target_dependencies(ur3e_mrc_robotiq
  rclcpp
  std_msgs
  ur_rtde
)


install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}/
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(
  TARGETS ur3e_mrc_enme480_topics
  DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include)

install(
  TARGETS ur3e_mrc_enme480_ctrl
  DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include)

install(
  TARGETS ur3e_mrc_robotiq
  DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

target_link_libraries(ur3e_mrc_enme480_topics "${cpp_typesupport_target}")
target_link_libraries(ur3e_mrc_enme480_ctrl "${cpp_typesupport_target}")
target_link_libraries(ur3e_mrc_robotiq "${cpp_typesupport_target}" ur_rtde::rtde)

ament_package()
