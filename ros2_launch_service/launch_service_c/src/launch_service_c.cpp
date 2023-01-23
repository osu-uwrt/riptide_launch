#include <fstream>
#include <csignal>

#include "launch_service_c/launch_manager_c.h"

int main(int argc, char ** argv)
{
  // Look for super secret flag to see if this is a child process
  int child_flag_index = -1;
  for (int i = 0; i < argc; ++i) {
    if (std::strcmp(SUPER_SECRET_FLAG.c_str(), argv[i]) == 0) {
      child_flag_index = i;
      break;
    }
  }

  if (child_flag_index >= 0) {
    // CHILD PROCESS
    std::cout << "Starting child. . .\n";

    // Check if there are enough arguments passed through for there to be a launch argument
    if (argc >= child_flag_index + 2) {
      // Verify file exists
      std::ifstream f(argv[child_flag_index + 1]);

      if (f.good()) {
        // Execute Python.
        exec_python(argv[child_flag_index + 1]);

      } else {
        std::cerr << "File \"" << argv[child_flag_index + 1] << "\" does not exist.\n";
        exit(EXIT_FAILURE);
      }

    } else {
      std::cerr << "Not enough arguments supplied for there to exist a launch path. . .\n";
      exit(EXIT_FAILURE);
    }
  } else {

    // Ignore SIGCHLD signals, so the child completely dies without much of a fuss.
    struct sigaction sigchld_ignore;
    sigchld_ignore.sa_handler = SIG_IGN;
    sigchld_ignore.sa_flags = 0; // or SA_RESTART
    sigemptyset(&sigchld_ignore.sa_mask);
    sigaction(SIGCHLD, &sigchld_ignore, NULL);

    // PARENT PROCESS
    // init ros node
    std::cout << "Starting parent. . .\n";
    rclcpp::init(argc, argv);

    // Create the node and spin it 
    auto driver = std::make_shared<launch_manager::LaunchManager>();
    rclcpp::spin(driver);

    // the node has ben called to shutdown, so rclcpp needs to be shut down as well
    rclcpp::shutdown();
  }
  return 0;
}