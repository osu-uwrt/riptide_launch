#include "launch_service_c/launch_manager_c.h"

#define MAX_HOST_LEN 300

using namespace launch_manager;
using namespace std::placeholders;
using namespace std::chrono_literals;

LaunchManager::LaunchManager() : Node("launch_manager"){
    

    RCLCPP_INFO_STREAM(get_logger(), "Binding to hostname '" << hostname << "'");

    // create the action server
    bringup_start = rclcpp_action::create_server<launch_msgs::action::BringupStart>(
      this,
      hostname + "/bringup_start",
      std::bind(&LaunchManager::handle_bringup_goal, this, _1, _2),
      std::bind(&LaunchManager::handle_bringup_cancel, this, _1),
      std::bind(&LaunchManager::handle_bringup_accepted, this, _1));

    // create the service server to stop things

    // create the alive topic
    bringup_status = create_publisher<launch_msgs::msg::ListLaunch>(hostname + "/launch_status", rclcpp::SystemDefaultsQoS());

    // create the status publish timer
    publish_timer = create_wall_timer(2s, std::bind(&LaunchManager::pub_timer_callback, this));
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

    // make sure this is the child thread created during the fork
    if(pid == 0) {
        // create the package path
        std::string packagePath;
        if(goal_handle->get_goal()->launch_package.find('/') != std::string::npos)
            packagePath = goal_handle->get_goal()->launch_package;
        else
            packagePath = ament_index_cpp::get_package_share_directory(
                goal_handle->get_goal()->launch_package) + "/launch";
        
        // invoke the interpreter
        pybind11::scoped_interpreter guard{}; // start the interpreter and keep it alive
        pybind11::exec("from ros2launch.api import launch_a_launch_file"); // import the launch API
        std::string pyline = "launch_a_launch_file(launch_file_path='" + packagePath 
            + "/" + goal_handle->get_goal()->launch_file + "', launch_file_arguments=[])";
        pybind11::exec(pyline); // start the requested launch file

        std::cout << "Launch process Ended" << std::endl;

    } else {
        result->pid = pid;

        // begin startup monitoring here

        std::cout << "Parent thread completed" << std::endl;
        goal_handle->succeed(result);
    }
}

void LaunchManager::pub_timer_callback(){
    // create the vector of pids and resize it correctly
    std::vector<int16_t> pidList;
    pidList.reserve(bringup_listeners.size());

    // get the list of PIDs
    for(auto const& imap: bringup_listeners)
        pidList.push_back(imap.first);

    // create the message
    launch_msgs::msg::ListLaunch launchesMsg;
    launchesMsg.pids = pidList;

    // send the status
    bringup_status->publish(launchesMsg);
}

const std::string LaunchManager::get_hostname(){
    // retrieve the system hostname in hopefully MAX_HOST_LEN characters -1 for null term
    char hostCstr[MAX_HOST_LEN];
    ::gethostname(hostCstr, MAX_HOST_LEN);
    
    std::string hostnameInternal = hostCstr;

    // make sure we have a null termination
    if(hostnameInternal.length() >= MAX_HOST_LEN){
        hostnameInternal = "unknown_host";
        RCLCPP_WARN_STREAM(get_logger(), "Failed to discover system hostname, falling back to default, " << hostnameInternal);
    } else {
        // replace the dashes with dots
        std::replace(hostnameInternal.begin(), hostnameInternal.end(), '-', '_');
    }


}