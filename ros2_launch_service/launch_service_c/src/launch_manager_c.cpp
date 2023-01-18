#include "launch_service_c/launch_manager_c.h"

#define MAX_HOST_LEN 300

using namespace launch_manager;
using namespace std::placeholders;
using namespace std::chrono_literals;

LaunchManager::LaunchManager() : Node("launch_manager"){
    
    hostname = get_hostname();

    RCLCPP_INFO_STREAM(get_logger(), "Binding to hostname '" << hostname << "'");

    // create the action server to start launches
    bringup_start = rclcpp_action::create_server<launch_msgs::action::BringupStart>(
      this,
      hostname + "/bringup_start",
      std::bind(&LaunchManager::handle_bringup_goal, this, _1, _2),
      std::bind(&LaunchManager::handle_bringup_cancel, this, _1),
      std::bind(&LaunchManager::handle_bringup_accepted, this, _1));

    // create the action server to stop things
    bringup_end = rclcpp_action::create_server<launch_msgs::action::BringupEnd>(
        this,
        hostname + "/bringup_end",
        std::bind(&LaunchManager::handle_end_goal, this, _1, _2),
        std::bind(&LaunchManager::handle_end_cancel, this, _1),
        std::bind(&LaunchManager::handle_end_accepted, this, _1));

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
    // two processes at this point
    pid_t pid = fork();

    // make sure this is the child thread created during the fork
    if(pid == 0) {
        RCLCPP_INFO(get_logger(), "Child thread starting");

        // create the package path
        std::string packagePath;
        if(goal_handle->get_goal()->launch_package.find('/') != std::string::npos)
            packagePath = goal_handle->get_goal()->launch_package;
        else
            packagePath = ament_index_cpp::get_package_share_directory(
                goal_handle->get_goal()->launch_package) + "/launch";

        // VERY IMPORTANT CALL -- need to remove signal handlers against
        // the child otherwise we cannot sigint the child
        rclcpp::uninstall_signal_handlers();

        // may need to also clear the ros data 
        
        // invoke the interpreter
        pybind11::scoped_interpreter guard{}; // start the interpreter and keep it alive
        pybind11::exec("from ros2launch.api import launch_a_launch_file"); // import the launch API
        std::string pyline = "launch_a_launch_file(launch_file_path='" + packagePath 
            + "/" + goal_handle->get_goal()->launch_file + "', launch_file_arguments=[])";

        pybind11::exec(pyline); // start the requested launch file

        std::cout << "Launch process Ended" << std::endl;
        rclcpp::shutdown();
        exit(0);

    } else {
        RCLCPP_INFO_STREAM(get_logger(), "Parent thread begin monitoring on PID " << pid);

        // work through each topic to make the list of topics to monitor
        std::vector<std::tuple<rclcpp::GenericSubscription::SharedPtr, std::shared_ptr<GenericSubCallback>>> genSubscrip;
        for (auto topic : goal_handle->get_goal()->topics){

            // determine the QOS from the info we were given
            rclcpp::QoS genSubQos = rclcpp::SensorDataQoS(); 
            switch(topic.qos_type){
                case launch_msgs::msg::TopicData::QOS_SENSOR_DATA:
                    genSubQos = rclcpp::SensorDataQoS();
                    break;

                case launch_msgs::msg::TopicData::QOS_SYSTEM_DEFAULT:
                    genSubQos = rclcpp::SystemDefaultsQoS();
                    break;

                default:
                    genSubQos = rclcpp::SystemDefaultsQoS();
                    RCLCPP_ERROR(get_logger(), "Unknown QOS type sent during request, "
                        "check that the QOS type matches the available options");
            }

            // create the Generic subscription and its counterpart info class
            auto genSubCb = std::make_shared<GenericSubCallback>(pid);
            auto genSub = create_generic_subscription(topic.name, topic.type_name, genSubQos, 
                std::bind(&GenericSubCallback::callback, genSubCb, _1));

            // add it to the subscriptions list
            genSubscrip.push_back(std::make_tuple(genSub, genSubCb));
        }

        // add the PID to the list of processes to track
        bringup_listeners.emplace(pid, genSubscrip);

        // now we can thread out to monitor the startup
        auto action_thread_ = std::thread{std::bind(&LaunchManager::monitor_child_start, this, _1, _2), pid, goal_handle};
        action_thread_.detach();
    }
}

void LaunchManager::monitor_child_start(
    pid_t pid, const std::shared_ptr<rclcpp_action::ServerGoalHandle<launch_msgs::action::BringupStart>> goal_handle){

    RCLCPP_INFO(get_logger(), "Launch entring monitoring state");

    auto result = std::make_shared<launch_msgs::action::BringupStart::Result>();
    result->pid = pid;

    // get the expected # of topics
    int topicCount = goal_handle->get_goal()->topics.size();

    // setup loop timeout and sleep time
    rclcpp::Rate checkRate(1s);
    auto startTime = get_clock()->now();
    int recievedCount = 0;

    // create the feedback message
    auto fbMsg = std::make_shared<launch_msgs::action::BringupStart::Feedback>();
    fbMsg->expected_topics = topicCount;

    // begin startup monitoring here
    while(startTime + startup_timeout > get_clock()->now()){
        recievedCount = 0;

        // figure out which of the subscribers we have data from and which we dont
        for(auto subscrip : bringup_listeners.at(pid)){
            recievedCount += std::get<1>(subscrip)->hasRecievedData()? 1 : 0;
        }

        // fill out and send the feedback message with current status
        fbMsg->completed_topics = recievedCount;
        goal_handle->publish_feedback(fbMsg);

        // check if the recieve count matches the expected count. if they do, bail the loop
        if(recievedCount == topicCount)
            break;

        // sleep a bit
        checkRate.sleep();
    } 

    // close out the subscribers and destroy them
    for(auto subscrip : bringup_listeners.at(pid)){
        // std::cout << std::get<0>(subscrip).use_count() << " " << std::get<1>(subscrip).use_count() << std::endl;

        std::get<0>(subscrip).reset();
        std::get<1>(subscrip).reset();

        // std::cout << std::get<0>(subscrip).use_count() << " " << std::get<1>(subscrip).use_count() << std::endl;
    }


    // check for a timeout vs a success
    if(recievedCount != topicCount){
        RCLCPP_ERROR(get_logger(), "Unable to determine startup due to timeout. ABORTING!");

        // send the child pid a sigint and check to make sure it wasnt already dead
        int err = kill(pid, SIGINT); 
        if(err == -1 && errno == ESRCH) {
            RCLCPP_ERROR(get_logger(), "Launch process died during startup...");
        }

        // abort the action
        goal_handle->abort(result);

    } else {
        RCLCPP_INFO(get_logger(), "Launch completed!");

        goal_handle->succeed(result);
    } 
}

rclcpp_action::GoalResponse LaunchManager::handle_end_goal (const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const launch_msgs::action::BringupEnd::Goal> goal) {
    // Ensure PID is within the bringup_listeners before accepting
    if (bringup_listeners.find(goal->pid) != bringup_listeners.end()) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    } else {
        RCLCPP_ERROR(get_logger(), "Process with PID %d is not a child of the launch monitor. Rejecting goal. . .", goal->pid);
        return rclcpp_action::GoalResponse::REJECT;
    }
}

rclcpp_action::CancelResponse LaunchManager::handle_end_cancel (const std::shared_ptr<rclcpp_action::ServerGoalHandle<launch_msgs::action::BringupEnd>> goal_handle) {
    // The processes are already finishing. No time to change your mind now.
    RCLCPP_WARN(get_logger(), "Rejecting cancellation of bringup_end goal for process with PID %d.", goal_handle->get_goal()->pid);
    return rclcpp_action::CancelResponse::REJECT;
}

void LaunchManager::handle_end_accepted (const std::shared_ptr<rclcpp_action::ServerGoalHandle<launch_msgs::action::BringupEnd>> goal_handle) {
    // Signal process to close
    int pid = goal_handle->get_goal()->pid;
    auto result = std::make_shared<launch_msgs::action::BringupEnd_Result>();

    RCLCPP_DEBUG(get_logger(), "Killing process with PID %d. . .", pid);
    int err = kill(pid, SIGINT);
    
    if (err == -1) {
        if (errno == ESRCH) {
            RCLCPP_ERROR(get_logger(), "Process with PID %d does not exist or is already terminating.", pid);
            goal_handle->abort(result);
        } else if (errno == EPERM) {
            RCLCPP_ERROR(get_logger(), "The monitor has no permission to kill process with PID %d.", pid);
            goal_handle->abort(result);
        }
    } else {
        bringup_listeners.erase(pid);
        goal_handle->succeed(result);
    }
}

void LaunchManager::pub_timer_callback(){
    // check for subscribers before doing any of this work
    // if no subscribers, dont publish
    if(bringup_status->get_subscription_count() == 0)
        return;

    // create the vector of pids and resize it correctly
    std::vector<int16_t> pidList;
    pidList.reserve(bringup_listeners.size());

    // get the list of PIDs
    for(auto const& imap : bringup_listeners)
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
    
    std::string hostnameInternal(hostCstr);

    // make sure we have a null termination
    if(hostnameInternal.length() >= MAX_HOST_LEN){
        hostnameInternal = "unknown_host";
        RCLCPP_WARN_STREAM(get_logger(), "Failed to discover system hostname, falling back to default, " << hostnameInternal);
    } else {
        // replace the dashes with underscores, because the spec doesnt like dashes
        std::replace(hostnameInternal.begin(), hostnameInternal.end(), '-', '_');
    }

    // kinda important.... without this strings raise a bad_alloc
    return hostnameInternal;
}

void GenericSubCallback::callback(std::shared_ptr<rclcpp::SerializedMessage> msg){
    hasRecieved = true;
    // std::cout << "got data in pid " << getpid() << std::endl;
}