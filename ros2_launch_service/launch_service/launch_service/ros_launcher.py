import subprocess, os, signal

from ament_index_python.packages import PackageNotFoundError
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from ros2launch.api import get_share_file_path_from_package
from ros2launch.api import is_launch_file
from ros2launch.api import MultipleLaunchFilesError

from launch_msgs.msg import LaunchID
from launch_msgs.srv import ListLaunch, StartLaunch, StopLaunch

def get_host():
    return os.uname()[1].replace('-', '_')

class LaunchNode(Node):
    def __init__(self, hostname) -> None:
        super().__init__(f'{hostname}_launch_srv')

        # callback groups for the executor
        self.service_group = ReentrantCallbackGroup()
        self.timer_group = ReentrantCallbackGroup()

        # service servers for executing callbacks
        self.start_srv = self.create_service(StartLaunch, f'{hostname}/start_launch', self.start_cb, callback_group=self.service_group)
        self.list_srv = self.create_service(ListLaunch, f'{hostname}/list_launch', self.list_cb, callback_group=self.service_group)
        self.stop_srv = self.create_service(StopLaunch, f'{hostname}/stop_launch', self.stop_cb, callback_group=self.service_group)

        # current launch ID to be used on the next startup
        self.start_id = 0

        # map for managing internal threads
        # structure has launch ID # as the key, and contains a tuple of LaunchThread and LaunchID
        self.launch_map = {}

        self.get_logger().info('Launch service started')

    def start_cb(self, req, resp):
        # at this point req contains two valid fields
        launch_file = str(req.launch_file)
        launch_pkg = str(req.package)

        launch_path = launch_file
        try:
            if not os.path.isfile(launch_file):
                # we were given a package and a file name
                # first we need to parse the location of the file
                launch_path = get_share_file_path_from_package(
                        package_name=launch_pkg,
                        file_name=launch_file)
            
            if not is_launch_file(launch_path):
                raise RuntimeError(f"launch file {launch_path} is not valid!")

        except PackageNotFoundError as exc:
            resp.started = False
            resp.error = "Package '{}' not found: {}".format(launch_pkg, exc)
            return 
        except (FileNotFoundError, MultipleLaunchFilesError, RuntimeError) as exc:
            resp.started = False
            resp.error = str(exc)
            return

        # build the LaunchID
        launch_id = LaunchID()
        launch_id.launch_id = self.start_id
        launch_id.status = LaunchID.RUNNING
        launch_id.package = launch_pkg
        launch_id.launch_file = launch_file
        self.start_id += 1

        # fill out the launch ID field in the response
        resp.formed_launch = launch_id

        # create and start the process
        self.get_logger().info(f'Launching {launch_path}')
        launch_context = subprocess.Popen(f'ros2 launch {launch_path}', 
            shell=True, env=dict(os.environ), preexec_fn=os.setsid)

        # add the entry in the dictonary
        self.launch_map[launch_id.launch_id] = (
            launch_id,
            launch_context
        )

        # fill out the resp
        resp.started = True
        resp.error = ''

        return resp

    def check_launch(self, launch_id):
        # check the launch process to see if it is still alive

        if launch_id in self.launch_map and self.launch_map[launch_id][0].status == LaunchID.RUNNING:
            # make sure the launch ID is valid
            launch_tup = self.launch_map[launch_id]

            try:
                # check to see if the process has exited.
                status = launch_tup[1].wait(timeout=0.01)

                if status == 0:
                    launch_tup[0].status = LaunchID.EXITED
                else:
                    launch_tup[0].status = LaunchID.ERRORED

            except subprocess.TimeoutExpired:
                # since we are just monitoring, ignore the error here
                pass

            return True


        else:
            # we discovered an invalid ID
            return False


    def list_cb(self, req, resp):
        # self.get_logger().info('Recieved query for launch listing')
        launches = []
        # work through the list of launches to show 
        for launch in self.launch_map.values():
            launch_id = launch[0]

            # self.get_logger().info(f'Checking launch file with ID {launch_id.launch_id}')

            self.check_launch(launch_id.launch_id)

            # self.get_logger().info(f'Current status {launch_id.status}')

            # filter it such that unknown in the request gives all, or another gives only that type
            if(req.status == LaunchID.UNKNOWN or req.status == launch_id.status):
                launches.append(launch_id)

        # self.get_logger().info('Retrieved all launch listings')
        
        resp.launches = launches
        return resp

    def kill_process(self, launch_id):
        launch = self.launch_map[launch_id]

        # used to stop this thread by sending a sigint
        os.killpg(os.getpgid(launch[1].pid), signal.SIGINT)

        # self.get_logger().info('interrupted process')
        try:
            # wait for the launch file to come down
            launch[1].wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            self.get_logger().warning('shutdown timed out, killing')
            # if we timed out, go ahead and kill the process
            # used to stop this thread by sending a sigint
            os.killpg(os.getpgid(launch[1].pid), signal.SIGTERM)

    def stop_cb(self, req, resp):
        # self.get_logger().info(f'Request to stop launch ID {req.launch_id}')

        # set default resp values
        resp.stopped = False
        resp.error = f'Could not find launch ID {req.launch_id}'

        if self.check_launch(req.launch_id):
            # self.get_logger().info('Launch exists, process checked for status')

            launch = self.launch_map[req.launch_id]

            # check to see if the launch ID exists
            if launch[0].status != LaunchID.RUNNING:
                # test if the launch has already exited
                # self.get_logger().info('Process already stopped')
                resp.stopped = False
                resp.error = f'Launch {req.launch_id} is already stopped with code {self.launch_map[req.launch_id][0].status}'

            else:
                # self.get_logger().info('Process not yet stopped. sending signal')
                # used to stop this thread by sending a sigint
                self.kill_process(req.launch_id)

                resp.stopped = True
                resp.error = ''

        return resp


    def closeout(self):
        # go through each thread and shut them down if they were in a running state
        for launch in self.launch_map.values():
            id = launch[0].launch_id

            # make sure the info is up to date on the process
            self.check_launch(id)

            # check if the thread is still running
            if launch[0].status == LaunchID.RUNNING:
                print(f'killing {id}')
                self.kill_process(id)
                

def main():
    rclpy.init()
    hostname = get_host()

    node = LaunchNode(hostname=hostname)

    # create the executro with 2 threads. one for each group
    exec = MultiThreadedExecutor(2)

    try:
        # spin the ros threads
        rclpy.spin(node, executor=exec)

    except KeyboardInterrupt:
        node.closeout()

    # when interrupted, shut down cleanly
    rclpy.shutdown()

if __name__ == '__main__':
    main()
