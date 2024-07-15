# Python 3 server example
from http.server import BaseHTTPRequestHandler, HTTPServer
import os, sys, subprocess, json, yaml, signal, fcntl
from typing import List
import enum
import re
import time
import signal

from ament_index_python.packages import get_package_share_directory

import uuid

SUPER_SECRET_LAUNCH_FLAG = "--super-secret-forking-flag"
SUPER_SECRET_MONITOR_FLAG = "--super-secret-monitoring-flag"
TIMEOUT = 15.0

hostName = "0.0.0.0"
serverPort = 8080

PAGE_HTML_PATH = os.path.join(get_package_share_directory("remote_launch"), "pages", "page.html")

class LaunchState(enum.Enum):
    STOPPED = 0
    RUNNING = 1
    STOPPING = 2
    STOPPING_ERROR = 3
    ERROR = 4

class LaunchData:
    class LaunchTopic:
        topic_re = re.compile(r"^(\/\w+)+$")  # Match at least 1 group of alpha-numeric chars starting with a slash
        type_re = re.compile(r"^[a-zA-Z]\w*\/msg\/[a-zA-Z]\w*$")  # Match c_identifier/msg/c_identifier

        def __init__(self, topic_data):
            self.name: str = topic_data[0]
            if not re.match(self.topic_re, self.name):
                raise ValueError(f"Invalid Topic Name '{self.name}': Must start with / and be a valid topic name")
            self.type: str = topic_data[1]
            if not re.match(self.type_re, self.type):
                raise ValueError(f"Invalid Topic Type '{self.type}': Must be in the form package/msg/MsgName")

            if topic_data[2] == "0":
                self.is_sensor_data = False
            elif topic_data[2] == "1":
                self.is_sensor_data = True
            else:
                raise ValueError(f"Invalid is_sensor_data param for {self.name}: " + \
                                 f"must be either \"0\" or \"1\", not \"{topic_data[2]}\"")

            self.seen = False

    def __init__(self, launch: dict):
        self.id = str(uuid.uuid4())
        self.friendly_name: str = launch["package"] + "/" + launch["file"]
        self.package: str = launch["package"]
        self.file: str = launch["file"]
        self.state = LaunchState.STOPPED
        self.is_zombie = False
        self.monitored_topics: 'list[LaunchData.LaunchTopic]' = []
        if len(launch["topics"]) % 3 != 0:
            raise RuntimeError(f"Launch {self.friendly_name} has a malformed topic list")

        for i in range(0, len(launch["topics"]), 3):
            self.monitored_topics.append(self.LaunchTopic(launch["topics"][i:i+3]))

        self.args: str = launch["args"]
        self._monitor_subproc = None
        self._launch_subproc = None

    def _clear_monitored_topics(self):
        for topic in self.monitored_topics:
            topic.seen = False

    def refresh_state(self):
        if self._launch_subproc is None and self._monitor_subproc is None:
            return  # Nothing to do, both processes have been cleaned up

        if self.state == LaunchState.STOPPING or self.state == LaunchState.STOPPING_ERROR:
            # We're just waiting for the processes to clean up, once both are okay, then set state to stopped/error
            if self._launch_subproc is not None:
                status = self._launch_subproc.poll()
                if status is not None:
                    # Process is now dead and cleaned up, safe to delete reference
                    self._launch_subproc = None

            if self._monitor_subproc is not None:
                status = self._monitor_subproc.poll()
                if status is not None:
                    # Process dead, safe to remove reference
                    self._monitor_subproc = None

            if self._launch_subproc is None and self._monitor_subproc is None:
                # Both processes have cleaned up, we can now advance to the stopped state
                # (Resting state depends if launch died in error or not)
                self._clear_monitored_topics()
                if self.state == LaunchState.STOPPING:
                    self.state = LaunchState.STOPPED
                else:
                    self.state = LaunchState.ERROR

        else:
            assert self.state == LaunchState.RUNNING, f"Processes alive in unexpected state {self.state}"

            launch_status = self._launch_subproc.poll()
            if launch_status is not None:
                # child has died T_T
                print(f"[Remote Launch] Launch {self.friendly_name} died")
                self._launch_subproc = None

                if self._monitor_subproc is not None:
                    try:
                        self._monitor_subproc.send_signal(signal.SIGINT)
                    except:
                        # It's okay if we fail, that means the monitor quit since the last refresh
                        pass

                    self.state = LaunchState.STOPPING_ERROR
                else:
                    # Both monitor and child cleaned up, safe to go directly to error
                    self._clear_monitored_topics()
                    self.state = LaunchState.ERROR


            elif self._monitor_subproc is not None:
                # Monitor still running, need to update the topic list

                # Handle the byte indices coming in
                monitor_out = self._monitor_subproc.stdout.read(1)
                while monitor_out is not None and len(monitor_out) > 0:
                    topic_idx = monitor_out[0]
                    if topic_idx < len(self.monitored_topics):
                        self.monitored_topics[topic_idx].seen = True
                    else:
                        print(f"ERROR: Unexpected index received on stdin: {topic_idx} ('{chr(topic_idx)}')")
                    monitor_out = self._monitor_subproc.stdout.read(1)

                # Handle monitor cleanup
                monitor_status = self._monitor_subproc.poll()
                if monitor_status is not None:
                    # Monitor terminated, safe to free if
                    self._monitor_subproc = None

                    # Make sure we received all the topics, if not the monitor must have crashed, and we should error
                    all_topics_seen = True
                    for topic in self.monitored_topics:
                        if not topic.seen:
                            all_topics_seen = False

                    if not all_topics_seen:
                        try:
                            self._launch_subproc.send_signal(signal.SIGINT)
                        except:
                            # It's okay if we fail, that means the launch also crashed
                            pass
                        self.state = LaunchState.STOPPING_ERROR

    def _gen_monitor_cmd_args(self):
        for topic in self.monitored_topics:
            yield topic.name
            yield topic.type
            yield "1" if topic.is_sensor_data else "0"

    @property
    def can_launch(self):
        return self.state == LaunchState.STOPPED or self.state == LaunchState.ERROR

    @property
    def can_terminate(self):
        return self.state == LaunchState.RUNNING

    def perform_launch(self):
        if self._monitor_subproc is not None or self._launch_subproc is not None:
            raise RuntimeError("Attempting to start a launch when launch already in progress")

        # launch fork
        popen_args = ["/proc/self/exe", __file__, SUPER_SECRET_LAUNCH_FLAG,
                      self.package, self.file]
        popen_args.extend(self.args)
        launch_subproc = subprocess.Popen(popen_args)

        # pipe stdout to this process with noblock
        popen_args = ["/proc/self/exe", __file__, SUPER_SECRET_MONITOR_FLAG]
        popen_args.extend(self._gen_monitor_cmd_args())
        monitor_subproc = subprocess.Popen(popen_args, stdout=subprocess.PIPE)
        flags = fcntl.fcntl(monitor_subproc.stdout.fileno(), fcntl.F_GETFL)
        fcntl.fcntl(monitor_subproc.stdout.fileno(), fcntl.F_SETFL, flags | os.O_NONBLOCK)

        # Update internal state
        self._clear_monitored_topics()
        self.state = LaunchState.RUNNING
        self._monitor_subproc = monitor_subproc
        self._launch_subproc = launch_subproc

    def term_launch(self):
        if self._launch_subproc is None:
            raise RuntimeError("Attempting to kill an already cleaned up launch")

        self.state = LaunchState.STOPPING

        print(f"[Remote Launch] Interrupting {self.friendly_name}")
        try:
            self._launch_subproc.send_signal(signal.SIGINT)
        except:
            # It's okay if we fail, that means the process died just as we sent the signal, set it to error though
            self.state = LaunchState.STOPPING_ERROR

        if self._monitor_subproc is not None:
            try:
                self._monitor_subproc.send_signal(signal.SIGINT)
            except:
                # Same as above
                self.state = LaunchState.STOPPING_ERROR

    @property
    def is_dead_zombie(self) -> bool:
        return self.is_zombie and self._monitor_subproc is None and self._launch_subproc is None

def json_encode_default(o):
    if isinstance(o, LaunchState):
        return o.name.lower()
    elif isinstance(o, LaunchData):
        return {
            "id": o.id,
            "friendly_name": o.friendly_name,
            "package": o.package,
            "file": o.file,
            "state": o.state,
            "monitored_topics": o.monitored_topics,
            "args": o.args,
            "is_zombie": o.is_zombie
        }
    elif isinstance(o, LaunchData.LaunchTopic):
        return {
            "name": o.name,
            "type": o.type,
            "is_sensor_data": o.is_sensor_data,
            "seen": o.seen
        }
    else:
        raise TypeError(f'Object of type {o.__class__.__name__} '
                        f'is not JSON serializable')

class ParentServer:
    def __init__(self) -> None:
        self.launches: 'list[LaunchData]' = []
        self.current_file = ""

    def load_file(self, file_path: str):
        if file_path == self.current_file:
            return

        # Mark all launches as zombies and ask them to terminate if running
        for launch in self.launches:
            launch.is_zombie = True
            launch.refresh_state()
            if launch.can_terminate:
                launch.term_launch()
        # Purge any dead ones
        self._cleanup_dead_zombies()

        # Populate with the new launch data
        self.current_file = file_path
        with open(file_path, 'r') as file:
            launch_data = yaml.safe_load(file)["launches"]
            for launch in launch_data:
                self.launches.append(LaunchData(launch))

    def _cleanup_dead_zombies(self):
        self.launches[:] = [inst for inst in self.launches if not inst.is_dead_zombie]

    def do_main(parent_self):
        class MyServer(BaseHTTPRequestHandler):
            def __init__(self, request, client_address, server) -> None:
                super().__init__(request, client_address, server)

            # can it!
            def log_message(self, msg, * args, ** kwargs):
                pass

            def do_POST(self):
                req_path = self.path.split("?")[0]
                if "?" in self.path:
                    i = self.path.index ( "?" ) + 1
                    params = dict ( [ tuple ( p.split("=") ) for p in self.path[i:].split ( "&" ) ] )
                else:
                    params = {}

                # this route allows spawning the launch
                if req_path == "/start_launch":
                    try:
                        launch_inst = next(x for x in parent_self.launches if x.id == params["id"])
                    except StopIteration:
                        self.send_error(400)
                        return

                    if launch_inst.can_launch:
                        launch_inst.perform_launch()
                        self.send_response(200)
                        self.send_header("Content-type", "text/html")
                        self.end_headers()
                    else:
                        self.send_error(400)
                        return

                # this route allows stopping a launch
                elif req_path == "/stop_launch":
                    try:
                        launch_inst = next(x for x in parent_self.launches if x.id == params["id"])
                    except StopIteration:
                        self.send_error(400)
                        return

                    if launch_inst.can_terminate:
                        launch_inst.term_launch()
                        self.send_response(200)
                        self.send_header("Content-type", "text/html")
                        self.end_headers()
                    else:
                        self.send_error(400)
                        return

                # this route allows stopping a launch
                elif req_path == "/load_launch":
                    pack_path = get_package_share_directory("remote_launch")
                    launch_filename = params["file"].replace('/', '')  # Path sanitization
                    launch_yaml = os.path.join(pack_path, "launches", launch_filename)

                    print(f"[Remote Launch] Loading launch definition {launch_yaml}")

                    if(os.path.isfile(launch_yaml)):
                        parent_self.load_file(launch_yaml)
                        self.send_response(200)

                    else:
                        self.send_error(400)
                        return

                    self.send_header("Content-type", "text/html")
                    self.end_headers()

                elif req_path == "/restart":
                    self.send_response(200)
                    self.send_header("Content-type", "text/html")
                    self.end_headers()

                    subprocess.Popen(["sudo", "reboot", "now"])

                else:
                    self.send_error(404)
                    return



            def do_GET(self):
                req_path = self.path.split("?")[0]

                # this route serves the page
                if self.path == "/":
                    self.send_response(200)
                    self.send_header("Content-type", "text/html")
                    self.end_headers()
                    with open(PAGE_HTML_PATH, "rb") as f:
                        self.wfile.write(f.read())
                    return

                # this route allows the webserver to retrieve state
                elif req_path == "/status":
                    self.send_response(200)
                    self.send_header("Access-Control-Allow-Origin", "*")
                    self.send_header("Content-type", "application/json")
                    self.end_headers()

                    # Refresh the state of all the launch instances
                    for launch_inst in parent_self.launches:
                        launch_inst.refresh_state()

                    # Clean up any zombies
                    parent_self._cleanup_dead_zombies()

                    self.wfile.write(json.dumps(parent_self.launches, default=json_encode_default).encode())

                # retrives availiable files
                elif req_path == "/launches":
                    pack_path = get_package_share_directory("remote_launch")
                    launch_entries = os.listdir(os.path.join(pack_path, "launches"))
                    files = [x for x in launch_entries if x.endswith('.yaml') or x.endswith('.yml')]

                    self.send_response(200)
                    self.send_header("Access-Control-Allow-Origin", "*")
                    self.send_header("Content-type", "application/json")
                    self.end_headers()

                    self.wfile.write(json.dumps({"files": files}, default=json_encode_default).encode())

                # retrives availiable files
                elif req_path == "/launch":
                    self.send_response(200)
                    self.send_header("Access-Control-Allow-Origin", "*")
                    self.send_header("Content-type", "application/json")
                    self.end_headers()

                    self.wfile.write(json.dumps({"file": os.path.basename(parent_self.current_file)},
                                                default=json_encode_default).encode())

                else:
                    self.send_error(404)
                    return




        webServer = HTTPServer((hostName, serverPort), MyServer)
        print("[Remote Launch] Server started http://%s:%s" % (hostName, serverPort))
        try:
            webServer.serve_forever()
        except KeyboardInterrupt:
            print("[Remote Launch] Keyboard Interrupt... Cleaning up child nodes")

        # Don't let control c kill it so we don't leave stragglers
        # If you REALLY want to kill it, use `ps f` and `kill` to make sure everything is killed
        signal.signal(signal.SIGINT, signal.SIG_IGN)

        webServer.server_close()

        # Mark all child launches to di
        for launch in parent_self.launches:
            launch.is_zombie = True
            launch.refresh_state()
            if launch.can_terminate:
                launch.term_launch()

        # Keep looping until all launches are dead
        while len(parent_self.launches) > 0:
            for launch in parent_self.launches:
                launch.refresh_state()
            parent_self._cleanup_dead_zombies()
            time.sleep(0.05)

        print("[Remote Launch] Gracefully Terminated (All Child Launches Cleaned Up)")


# monitor process stuff
class ChildMonitorCallback:
        def __init__(self, parent, index: int) -> None:
            self.parent = parent
            self.index = index
            self.has_called = False

        def callback(self, msg):
            if(not self.has_called):
                sys.stdout.buffer.write(bytearray([self.index]))
                sys.stdout.buffer.flush()
                self.has_called = True
                self.parent.remaining_topics -= 1
                if self.parent.remaining_topics < 1:
                    exit(0)

def do_child_monitor():
    import rclpy
    from rclpy.node import Node

    class ChildMonitor(Node):
        def __init__(self, node_name: str, *, context = None, cli_args: List[str] = None, namespace: str = None, use_global_arguments: bool = True, enable_rosout: bool = True, start_parameter_services: bool = True, parameter_overrides = None, allow_undeclared_parameters: bool = False, automatically_declare_parameters_from_overrides: bool = False) -> None:
            super().__init__(node_name, context=context, cli_args=cli_args, namespace=namespace, use_global_arguments=use_global_arguments, enable_rosout=enable_rosout, start_parameter_services=start_parameter_services, parameter_overrides=parameter_overrides, allow_undeclared_parameters=allow_undeclared_parameters, automatically_declare_parameters_from_overrides=automatically_declare_parameters_from_overrides)
            self.monitor_subs = list()

        def start_monitors(self, topics: list):
            self.remaining_topics = len(topics)
            for i in range(len(topics)):
                topic_name = topics[i][0]
                topic_type = str(topics[i][1]).split("/")
                topic_qos = "qos_profile_system_default" if int(topics[i][2]) == 0 else "qos_profile_sensor_data" if int(topics[i][2]) == 1 else None
                if topic_qos is None:
                    raise RuntimeError("Invalid QOS: Must be 1 for sensor data, 0 for system default")

                print(f"[Launch Monitor] Monitoring {topics[i][0]} of type {topics[i][1]} with QOS {topic_qos}", file=sys.stderr)

                # make the subscription
                sub_cb = ChildMonitorCallback(self, i)

                from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default

                sub = None
                exec(f"from {topic_type[0]}.{topic_type[1]} import {topic_type[2]}", locals())
                exec(f"sub = self.create_subscription({topic_type[2]}, '{topic_name}', sub_cb.callback, {topic_qos})", locals())

                self.monitor_subs.append((sub, sub_cb))

    rclpy.init()

    # chack to make sure we have correct args
    if len(sys.argv[2:]) % 3 != 0 or len(sys.argv[2:]) == 0:
        print("[Launch Monitor] ERROR! - Malformed launch args passed to monitor child!")
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


# flag handlers and launch integration
def main():

    # Handle the launch startup
    if len(sys.argv) > 2 and sys.argv[1] == SUPER_SECRET_LAUNCH_FLAG:
        os.setpgrp()  # We only want control C to go to the remote launch server

        from ros2launch.api import launch_a_launch_file
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

        print(f"[Child Launch] args: {args}")

        # launch the actual launch file with the CLI api
        launch_a_launch_file(
            launch_file_path=launch_file,
            launch_file_arguments=args
        )

    # if we detect the monitor flag launch the child monitor
    elif len(sys.argv) > 1 and sys.argv[1] == SUPER_SECRET_MONITOR_FLAG:
        os.setpgrp()  # We only want control C to go to the remote launch server

        # Child monitor is in its own function so the parent doesn't even know what ROS is
        do_child_monitor()

    # last resort (no args) is webserver
    else:
        server = ParentServer()
        pack_path = get_package_share_directory("remote_launch")
        server.do_main()



if __name__ == '__main__':
    main()
