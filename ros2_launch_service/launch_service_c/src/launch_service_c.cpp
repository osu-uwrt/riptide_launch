#include "launch_service_c/launch_manager_c.h"

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

    // init ros node
    rclcpp::init(argc, argv);

    // Create the node and spin it 
    auto driver = std::make_shared<launch_manager::LaunchManager>();
    rclcpp::spin(driver);

    // the node has ben called to shutdown, so rclcpp needs to be shut down as well
    rclcpp::shutdown();
    return 0;
  /*
  pybind11::scoped_interpreter guard{}; // start the interpreter and keep it alive
  pybind11::exec("from ros2launch.api import launch_a_launch_file"); // use the Python API
  std::string pyline = "";
  pyline += "launch_a_launch_file(";
  pyline += "launch_file_path='/opt/ros/humble/share/dummy_robot_bringup/launch/dummy_robot_bringup.launch.py',";
  pyline += "launch_file_arguments=[]";
  pyline += ")";

  pybind11::exec(pyline); // use the Python API
  return 0;
  */
}
