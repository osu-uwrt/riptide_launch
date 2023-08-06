# Python 3 server example
from http.server import BaseHTTPRequestHandler, HTTPServer
import time, os, sys, subprocess, json, yaml, signal, fcntl
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
TIMEOUT = 15.0

hostName = "0.0.0.0"
serverPort = 8080

PAGE_HTML_PATH = os.path.join(get_package_share_directory("remote_launch"), "pages", "page.html")

class ParentServer:
    def __init__(self, launches) -> None:
        self.launches = launches
        self.subprocs = {}
    
    def handle_launch_spawn(self, launch_pkg: str, launch_file: str, topics: List[str]):
        dict_name = launch_pkg+launch_file
        
        # launch fork
        popen_args = ["/proc/self/exe", __file__, SUPER_SECRET_LAUNCH_FLAG]
        popen_args.extend([launch_pkg, launch_file])
        popen_args.extend(self.launches[dict_name]["args"])
        launch_subproc = subprocess.Popen(popen_args)

        popen_args = ["/proc/self/exe", __file__, SUPER_SECRET_MONITOR_FLAG]
        popen_args.extend(topics)
        monitor_subproc = subprocess.Popen(popen_args, stdout=subprocess.PIPE)
        flags = fcntl.fcntl(monitor_subproc.stdout.fileno(), fcntl.F_GETFL)
        fcntl.fcntl(monitor_subproc.stdout.fileno(), fcntl.F_SETFL, flags | os.O_NONBLOCK)

        
        self.subprocs[dict_name] = {"launch_subproc": launch_subproc, "monitor_subproc": monitor_subproc}
        self.launches[dict_name]["running"] = True
        self.launches[dict_name]["error"] = False
        self.subprocs[dict_name]["topics_reporting"] = set()



    def handle_launch_term(self, launch_name: str, errored: bool):
        print(f"Interrupting {launch_name}")

        if(self.subprocs[launch_name]):
            self.subprocs[launch_name]["launch_subproc"].send_signal(signal.SIGINT)
            self.subprocs[launch_name]["monitor_subproc"].send_signal(signal.SIGINT)

        self.launches[launch_name]["running"] = False
        self.subprocs[launch_name] = None
        self.launches[launch_name]["error"] = errored

    def is_starting(self, launch_name: str):
        return self.launches[launch_name]["running"] and len(self.subprocs[launch_name]["topics_reporting"]) < self.launches[launch_name]["topics_count"]

    

    def do_main(parent_self):
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

                    launch_info = parent_self.launches[params["id"]]
                    if(not launch_info["running"]):
                        parent_self.handle_launch_spawn(launch_info["package"], launch_info["friendly_name"], launch_info["topics_data"])
                        self.send_response(200)
                    else:
                        self.send_response(500)

                    
                    self.send_header("Content-type", "text/html")
                    self.end_headers()

                # this route allows stopping a launch
                elif self.path.split("?")[0] == "/stop_launch":
                    i = self.path.index ( "?" ) + 1
                    params = dict ( [ tuple ( p.split("=") ) for p in self.path[i:].split ( "&" ) ] )

                    launch_info = parent_self.launches[params["id"]]
                    if(launch_info["running"]):
                        launch_name = launch_info["package"] + launch_info["friendly_name"]
                        parent_self.handle_launch_term(launch_name, False)  
                        self.send_response(200)

                    else:
                        self.send_response(500)

                    self.send_header("Content-type", "text/html")
                    self.end_headers()                  


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

                    # check on all the launch processes
                    for name in list(parent_self.subprocs.keys()):
                        # check if this is running
                        if parent_self.subprocs[name]:

                            # check if we died
                            launch_status = parent_self.subprocs[name]["launch_subproc"].poll()
                            if launch_status is not None:
                                # child has died T_T
                                print(f"Launch {name} died")
                                parent_self.handle_launch_term(name, True)

                            elif parent_self.is_starting(name):
                                # check for starting
                                monitor_out = parent_self.subprocs[name]["monitor_subproc"].stdout.read(1)
                                while monitor_out is not None and len(monitor_out) > 0:
                                        parent_self.subprocs[name]["topics_reporting"].add(monitor_out[0])
                                        monitor_out = parent_self.subprocs[name]["monitor_subproc"].stdout.read(1)


                            parent_self.launches[name]["topics_found"] = len(parent_self.subprocs[name]["topics_reporting"])


                    self.wfile.write(json.dumps(list(parent_self.launches.values())).encode())   

        webServer = HTTPServer((hostName, serverPort), MyServer)
        print("Server started http://%s:%s" % (hostName, serverPort))
        try:
            webServer.serve_forever()
        except KeyboardInterrupt:
            pass

        webServer.server_close()
    
    



# monitor process stuff
class ChildMonitorCallback:
        def __init__(self, index: int) -> None:
            self.index = index
            self.has_called = False

        def callback(self, msg):
            if(not self.has_called):
                sys.stdout.buffer.write(bytearray([self.index]))
                sys.stdout.buffer.flush()
                self.has_called = True


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

        print(f"args: {args}")
        
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
                    "package": launch["package"],
                    "error": False,
                    "running": False,
                    "topics_found": 0,
                    "topics_count": len(launch["topics"]) / 3,
                    "topics_data": launch["topics"],
                    "args": launch["args"]
                }

        server = ParentServer(launch_map)
        server.do_main()



if __name__ == '__main__':
    main()
