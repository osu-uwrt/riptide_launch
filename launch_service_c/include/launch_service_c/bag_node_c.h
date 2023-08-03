#pragma once

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

#include "rosbag2_transport/reader_writer_factory.hpp"
#include "rosbag2_transport/record_options.hpp"
#include "rosbag2_transport/recorder.hpp"

#include <launch_msgs/msg/topic_data.hpp>
#include <launch_msgs/srv/start_bag.hpp>

namespace launch_manager {
    int bag_record_main(const std::vector<std::string> & topics);
} // namespace launch_manager