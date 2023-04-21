#include "launch_service_c/bag_node_c.h"

#include <iostream>
#include <exception>

#include <rosbag2_storage/storage_options.hpp>

using std::placeholders::_1;

namespace launch_manager
{
    BagNode::BagNode(const std::string & name) : Node(name) {
    }

    int BagNode::subscribeTopics(const std::vector<launch_msgs::msg::TopicData> & topics,
                                  const std::string & bag_name){
        // setup the storage options
        rosbag2_storage::StorageOptions storage_options;
        storage_options.max_bagfile_duration = 30;
        storage_options.uri = bag_name;

        try{
            // make the bag writer
            writer = std::make_shared<rosbag2_cpp::Writer>();

            // set bag name from param
            writer->open(storage_options);

            // subscribe to the topics
            for(auto topic: topics){
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

                auto genSubCb = std::make_shared<GenericBagSubCallback>(
                    topic.name, topic.type_name, writer
                );

                auto genSub = create_generic_subscription(topic.name, topic.type_name, genSubQos, 
                        std::bind(&GenericBagSubCallback::callback, genSubCb, _1));

                // add them to the lists
                subscriptions.push_back(genSub);
                callbacks.push_back(genSubCb);
            }
        } catch (const std::runtime_error & e ){
            RCLCPP_FATAL(get_logger(), "Failed to open child bag %s\n%s", bag_name.c_str(), e.what());

            return launch_msgs::srv::StartBag::Response::ERR_OPENING_BAG;
        } catch (...) {
            RCLCPP_FATAL(get_logger(), "Failed to open child bag %s with unknown exception", bag_name.c_str());

            return launch_msgs::srv::StartBag::Response::ERR_UNKNOWN;
        }
        
        return launch_msgs::srv::StartBag::Response::ERR_NONE;
    }

    BagNode::~BagNode(){
        writer->~Writer();
    }

    int BagNode::main(const std::vector<std::string> & args){
        int exit_code = launch_msgs::srv::StartBag::Response::ERR_NONE;

        // create a node and run it
        rclcpp::init(0, nullptr);
        auto start_sec = std::to_string(int(rclcpp::Clock().now().seconds()));

        // first arg is the bag name
        const std::string bag_name = args.at(0);

        std::string node_name = bag_name;

        const size_t last_slash_idx = node_name.rfind('/', node_name.length());
        if (std::string::npos != last_slash_idx) {
            node_name = node_name.substr(last_slash_idx + 1, node_name.length() - last_slash_idx);
        };

        // Create the node and spin it
        auto bag = std::make_shared<BagNode>("bag_node_" + node_name);

        std::vector<launch_msgs::msg::TopicData> topics;
        for(size_t i = 1; i < args.size(); i+= 3){
            // make a new topic data
            launch_msgs::msg::TopicData topic;
            topic.name = args.at(i + 0);
            topic.type_name = args.at(i + 1);
            topic.qos_type = std::stoi(args.at(i + 2));

            topics.push_back(topic);
        }

        if(topics.size() < 1){
            RCLCPP_FATAL(bag->get_logger(), "No topics to bag given to child, process will terminate");

            exit_code = launch_msgs::srv::StartBag::Response::ERR_MISSING_TOPICS;
        } else {
            // setup the bag stuff
            exit_code = bag->subscribeTopics(topics, bag_name + "_" + start_sec);

            // make sure we init bag okay
            if(exit_code == launch_msgs::srv::StartBag::Response::ERR_NONE){
                // now spin the node until shutdown
                rclcpp::spin(bag);
            }
        }

        // shut down our rclcpp context cleanly
        rclcpp::shutdown();

        std::cout << "Bag shutdown complete" << std::endl;

        exit(exit_code);

        // cannot return from here otherwise i cant get process status for some reason ???
    }
} // namespace launch_manager
