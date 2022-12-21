#include "launch_service_c/launch_manager_c.h"

using namespace launch_manager;
using namespace std::placeholders;

/*
  pybind11::exec("from ros2launch.api import launch_a_launch_file"); // use the Python API
  std::string pyline = "";
  pyline += "launch_a_launch_file(";
  pyline += "launch_file_path='/opt/ros/humble/share/dummy_robot_bringup/launch/dummy_robot_bringup.launch.py',";
  pyline += "launch_file_arguments=[]";
  pyline += ")";

  pybind11::exec(pyline); // use the Python API
  return 0;
  */

LaunchManager::LaunchManager() : Node("launch_manager"){
    bringup_start = rclcpp_action::create_server<launch_msgs::action::BringupStart>(
      this,
      "bringup_start",
      std::bind(&LaunchManager::handle_bringup_goal, this, _1, _2),
      std::bind(&LaunchManager::handle_bringup_cancel, this, _1),
      std::bind(&LaunchManager::handle_bringup_accepted, this, _1));
}

rclcpp_action::GoalResponse LaunchManager::handle_bringup_goal (const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const launch_msgs::action::BringupStart::Goal> goal){
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse LaunchManager::handle_bringup_cancel (const std::shared_ptr<rclcpp_action::ServerGoalHandle<launch_msgs::action::BringupStart>> goal_handle) {
    return rclcpp_action::CancelResponse::ACCEPT;
}

void LaunchManager::handle_bringup_accepted (const std::shared_ptr<rclcpp_action::ServerGoalHandle<launch_msgs::action::BringupStart>> goal_handle) {    
        auto result = std::make_shared<launch_msgs::action::BringupStart::Result>();
        uint16_t pid = fork();
    if(pid == 0)
    {
        pybind11::scoped_interpreter guard{}; // start the interpreter and keep it alive
        pybind11::exec("from ros2launch.api import launch_a_launch_file"); // use the Python API
        std::string pyline = "";
        pyline += "launch_a_launch_file(";
        pyline += "launch_file_path='";
        if(goal_handle->get_goal()->launch_package.find('/') != std::string::npos)
            pyline += goal_handle->get_goal()->launch_package;
        else
            pyline += ament_index_cpp::get_package_share_directory(goal_handle->get_goal()->launch_package) + "/launch";
        pyline += "/";
        pyline += goal_handle->get_goal()->launch_file;
        pyline += "', launch_file_arguments=[]";
        pyline += ")";
        pybind11::exec(pyline); // use the Python API
    } else {
        result->pid = pid;
    }
        std::cout << "Process Completed" << std::endl;
        goal_handle->succeed(result);
        std::cout << "Process Ended" << std::endl;
}