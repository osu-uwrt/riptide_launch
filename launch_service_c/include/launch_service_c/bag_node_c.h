#pragma once

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

#include <rosbag2_cpp/writer.hpp>

#include <launch_msgs/msg/topic_data.hpp>
#include <launch_msgs/srv/start_bag.hpp>

namespace launch_manager {
    class GenericBagSubCallback {
        public:
        GenericBagSubCallback(const std::string & name, const std::string & type, std::shared_ptr<rosbag2_cpp::Writer> writer){
            this->name = name;
            this->type = type;
            this->writer = writer;
            clock = rclcpp::Clock();
        }

        void callback(std::shared_ptr<rclcpp::SerializedMessage> msg){
            writer->write(msg, name, type, clock.now());
        }

        private:
        std::string type, name;
        std::shared_ptr<rosbag2_cpp::Writer> writer;
        rclcpp::Clock clock;
    };

    class BagNode : public rclcpp::Node{
        public:
        BagNode(const std::string & name);

        int subscribeTopics(const std::vector<launch_msgs::msg::TopicData> & topics,
                             const std::string & bag_name);

        ~BagNode();

        static int main(const std::vector<std::string> & topics);

        private:
            std::vector<rclcpp::GenericSubscription::SharedPtr> subscriptions;
            std::vector<std::shared_ptr<GenericBagSubCallback>> callbacks;

            // bag writer
            std::shared_ptr<rosbag2_cpp::Writer> writer;
    };
} // namespace launch_manager