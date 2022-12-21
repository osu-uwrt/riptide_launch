# riptide_launch
The Riptide Launch repository is designed to execute statup tasks to be run onobard the vehicle. It provides full startup scripting as well as component wise startup scripting. Additionally the repository also contains packages for allowing remote startup of ROS packages on a remote system using a systemd service on the target computer.

|            |              |
|------------|--------------|
| OS Distro  | Ubuntu 22.04 |
| ROS Distro | ROS2 Humble  |

## riptide_bringup
Riptide Bringup is a high level package that allows starting up the riptide software stack in concert. The bringup.launch.py file coordinates all of the other ackages to bring the full robot autonomy stack online, and bring it down when finished. 

## ros2_launch_service
ROS2 Launch Service contains a set of executables that create a common interface for remotely launching or starting up systems on a remote target. For the riptide stack, this is used to start the vehicle without the need for an active SSH connection. It also allows our GUI interfaces to have control over the status of the subsystems of the robot.