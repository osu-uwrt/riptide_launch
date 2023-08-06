# Python 3 server example
from http.server import BaseHTTPRequestHandler, HTTPServer
import time, os, sys, subprocess, json, yaml
from typing import List

from ros2launch.api import launch_a_launch_file
from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.node import Node
from rclpy.context import Context
from rclpy.parameter import Parameter
import uuid

SUPER_SECRET_LAUNCH_FLAG = "--super-secret-forking-flag"
SUPER_SECRET_MONITOR_FLAG = "--super-secret-monitoring-flag"

hostName = "0.0.0.0"
serverPort = 8080

PAGE_HTML_PATH = os.path.join(get_package_share_directory("remote_launch"), "pages", "page.html")

class MyServer(BaseHTTPRequestHandler):
    def __init__(self, request, client_address, server) -> None:
        super().__init__(request, client_address, server)

    # can it!
    def log_message(self, msg, * args, ** kwargs):
        pass

    def do_POST(self):
        # this route allows spawning the launch
        if self.path.split("?")[0] == "/start_launch":
            i = self.path.index ( "?" ) + 1
            params = dict ( [ tuple ( p.split("=") ) for p in self.path[i:].split ( "&" ) ] )

    


        # this route allows stopping a launch
        elif self.path.split("?")[0] == "/stop_launch":
            i = self.path.index ( "?" ) + 1
            params = dict ( [ tuple ( p.split("=") ) for p in self.path[i:].split ( "&" ) ] )


    def do_GET(self):
        # this route serves the page
        if self.path == "/":
            self.send_response(200)
            self.send_header("Content-type", "text/html")
            self.end_headers()
            with open(PAGE_HTML_PATH, "rb") as f:
                self.wfile.write(f.read()) 

        # this route allows the webserver to retrieve state
        elif self.path == "/status":
            self.send_response(200)
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Content-type", "application/json")
            self.end_headers()

            data = [{
                "id": "mything",
                "friendly_name": "My Thing",
                "error": "true",
                "running": "true",
                "topics_found": 2,
                "topics_count": 8
            }]

            self.wfile.write(json.dumps(data).encode())   
    
    def handle_launch_spawn():
        # launch fork
        fork_val = os.fork() 
        if fork_val == 0:
            launch_subproc = subprocess.Popen(["/proc/self/exe", __file__, SUPER_SECRET_LAUNCH_FLAG, "dummy_robot_bringup", "dummy_robot_bringup.launch.py"])

        elif fork_val < 0:
            print("Fork failed to spawn launch ")

        else:
            # monitor fork
            fork_val = os.fork() 
            if fork_val == 0:
                monitor_subproc = subprocess.Popen(["/proc/self/exe", __file__, SUPER_SECRET_MONITOR_FLAG, "/rosout", "rcl_interfaces/msg/Log", "0"])

            elif fork_val < 0:
                print("Fork failed to spawn monitor ")



# monitor process stuff
class ChildMonitorCallback:
        def __init__(self, index: int) -> None:
            self.index = index

        def callback(self, msg):
            print(self.index)

class ChildMonitor(Node):
    def __init__(self, node_name: str, *, context: Context = None, cli_args: List[str] = None, namespace: str = None, use_global_arguments: bool = True, enable_rosout: bool = True, start_parameter_services: bool = True, parameter_overrides: List[Parameter] = None, allow_undeclared_parameters: bool = False, automatically_declare_parameters_from_overrides: bool = False) -> None:
        super().__init__(node_name, context=context, cli_args=cli_args, namespace=namespace, use_global_arguments=use_global_arguments, enable_rosout=enable_rosout, start_parameter_services=start_parameter_services, parameter_overrides=parameter_overrides, allow_undeclared_parameters=allow_undeclared_parameters, automatically_declare_parameters_from_overrides=automatically_declare_parameters_from_overrides)
        self.monitor_subs = list()

    def start_monitors(self, topics: list):
        for i in range(len(topics)):
            topic_name = topics[i][0]
            topic_type = str(topics[i][1]).split("/")
            topic_qos = "qos_profile_system_default" if topics[i][2] == 0 else "qos_profile_sensor_data"

            print(f"Monitoring {topics[i][0]} of type {topics[i][1]} with QOS {topics[i][2]}")

            # make the subscription
            sub_cb = ChildMonitorCallback(i)

            from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default

            sub = None
            exec(f"from {topic_type[0]}.{topic_type[1]} import {topic_type[2]}", locals())
            exec(f"sub = self.create_subscription({topic_type[2]}, '{topic_name}', sub_cb.callback, {topic_qos})", locals())

            self.monitor_subs.append((sub, sub_cb))



# flag handlers and launch integration
def main():

    # Handle the launch startup
    if len(sys.argv) > 2 and sys.argv[1] == SUPER_SECRET_LAUNCH_FLAG:
        arg_begin_idx = 3

        # resolve the package path
        launch_pack = sys.argv[2]
        launch_file = ""
        if "/" in launch_pack:
            launch_file = launch_pack
        else:
            pack_path = get_package_share_directory(launch_pack)
            launch_file = os.path.join(pack_path, "launch", sys.argv[3])
            arg_begin_idx += 1

        # look for args now
        args = sys.argv[arg_begin_idx :]
        
        # launch the actual launch file with the CLI api
        launch_a_launch_file(
            launch_file_path=launch_file,
            launch_file_arguments=args
        )

    # if we detect the monitor flag launch the child monitor
    elif len(sys.argv) > 1 and sys.argv[1] == SUPER_SECRET_MONITOR_FLAG:
        rclpy.init()

        # chack to make sure we have correct args
        if len(sys.argv[2:]) % 3 != 0 or len(sys.argv[2:]) == 0:
            print("Malformed launch args passed to monitor child!")
            return
        
        # build the topic info to send to the monitor node
        topics = []
        for i in range(2, len(sys.argv), 3):
            topics.append((sys.argv[i], sys.argv[i+1], sys.argv[i+2]))

        # prepare the monitor
        monitor = ChildMonitor(f"monitor_{uuid.uuid4().hex}")
        monitor.start_monitors(topics)

        # free run the monitor until shutdown
        rclpy.spin(monitor)
        rclpy.shutdown()

    # last resort (no args) is webserver
    else:
        webServer = HTTPServer((hostName, serverPort), MyServer)
        print("Server started http://%s:%s" % (hostName, serverPort))

        pack_path = get_package_share_directory("remote_launch")
        launch_yaml = os.path.join(pack_path, "launches", "talos_launch.yaml")


        launch_map = {}
        with open(launch_yaml, 'r') as file:
            launch_data = yaml.safe_load(file)["launches"]
            for launch in launch_data:
                full_name = launch["package"]+launch["file"]
                launch_map[full_name] = {
                    "id": full_name,
                    "friendly_name": launch["file"],
                    "error": False,
                    "running": False,
                    "topics_found": 0,
                    "topics_count": len(launch["topics"]) / 3,
                    "topics_data": launch["topics"]
                }

        print(launch_map)

        try:
            webServer.serve_forever()
        except KeyboardInterrupt:
            pass

        webServer.server_close()


if __name__ == '__main__':
    main()
