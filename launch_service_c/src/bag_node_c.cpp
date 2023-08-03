#include "launch_service_c/bag_node_c.h"

#include <iostream>
#include <exception>
#include <filesystem>

#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_cpp/plugins/plugin_utils.hpp>
#include <rosbag2_compression/base_compressor_interface.hpp>

using std::placeholders::_1;

namespace launch_manager
{

    int bag_record_main(const std::vector<std::string> &args)
    {
        int exit_code = launch_msgs::srv::StartBag::Response::ERR_NONE;

        // init rclcpp
        rclcpp::init(0, nullptr);

        auto start_sec = std::to_string(int(rclcpp::Clock().now().seconds()));

        // first arg is the bag name
        const std::string bag_name = args.at(0);

        std::string node_name = bag_name;

        const size_t last_slash_idx = node_name.rfind('/', node_name.length());
        if (std::string::npos != last_slash_idx)
        {
            node_name = node_name.substr(last_slash_idx + 1, node_name.length() - last_slash_idx);
        };

        std::vector<launch_msgs::msg::TopicData> topics;
        for (size_t i = 1; i < args.size(); i += 3)
        {
            // make a new topic data
            launch_msgs::msg::TopicData topic;
            topic.name = args.at(i + 0);
            topic.type_name = args.at(i + 1);
            topic.qos_type = std::stoi(args.at(i + 2));

            topics.push_back(topic);
        }

        if (topics.size() < 1)
        {
            std::cout << "No topics to bag given to child, process will terminate" << std::endl;

            exit_code = launch_msgs::srv::StartBag::Response::ERR_MISSING_TOPICS;
        }
        else
        {
            // setup the storage options to split every 30s or if a single db exceeds 3.5GB
            rosbag2_storage::StorageOptions storage_options;
            storage_options.max_bagfile_duration = 30;
            storage_options.max_bagfile_size = 3.5e9; 
            storage_options.uri = bag_name + "_" + start_sec;

            rosbag2_transport::RecordOptions record_options;

            // setup compression
            record_options.compression_mode = "file"; // can be none or message as well
            record_options.compression_format = "zstd";
            record_options.compression_queue_size = 10;
            record_options.compression_threads = 2;
            record_options.rmw_serialization_format = std::string(rmw_get_serialization_format());
            record_options.include_hidden_topics = true; // shut up!

            // setup topics
            std::vector<std::string> topic_names = {};
            for(auto topic: topics){
                topic_names.push_back(topic.name);
            }
            record_options.topics = topic_names;

            try{
                // now setup the recorder
                auto writer = rosbag2_transport::ReaderWriterFactory::make_writer(record_options);
                auto recorder = std::make_shared<rosbag2_transport::Recorder>(
                std::move(writer), storage_options, record_options, node_name);

                // start the recording
                recorder->record();

                rclcpp::spin(recorder);

                // TODO eventually add this when the api adds it
                // recorder->stop();
                
            }
            catch (const std::exception &e)
            {
                std::cout << "Failed to open child bag " << bag_name.c_str() << std::endl;
                std::cout << e.what() << std::endl;

                exit_code = launch_msgs::srv::StartBag::Response::ERR_OPENING_BAG;
            }
            catch (...)
            {
                std::cout << "Failed to open child bag " << bag_name.c_str() << " with unknown exception errno: " << errno << std::endl;

                exit_code = launch_msgs::srv::StartBag::Response::ERR_UNKNOWN;
            }

            exit_code = launch_msgs::srv::StartBag::Response::ERR_NONE;
        }

        // shut down our rclcpp context cleanly
        rclcpp::shutdown();

        std::cout << "Bag shutdown complete" << std::endl;

        exit(exit_code);

        // cannot return from here otherwise i cant get process status for some reason ???
    }
} // namespace launch_manager
