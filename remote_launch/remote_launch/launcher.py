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

class LinuxProcess:
    @staticmethod
    def lookup_state(state: str):
        state_map = {
            "R": "Running",
            "S": "Sleeping",
            "D": "Uninterruptable (Disk) Sleep",
            "Z": "Zombie",
            "T": "Stopped",
            "t": "Tracing Stop",
            "W": "Paging",
            "X": "Dead",
            "x": "Dead",
            "K": "Wakekill",
            "W": "Waking",
            "P": "Parked",
            "I": "Idle"
        }
        if state in state_map:
            return state_map[state]
        else:
            return f"Unknown ({state})"

    def __init__(self, pid, stat_str):
        stat_list = stat_str.split(' ')
        # Need to do some funky reading in case there's a space in the filename path
        self.pid = int(stat_list[0])
        if self.pid != pid:
            raise RuntimeError("Linux reported stats for pid that we did not request!")
        self.children: 'list[LinuxProcess]' = []
        self._args = None
        self._process_str = None
        self.comm = ' '.join(stat_list[1:-50]).lstrip('(').rstrip(')')
        self.state = self.lookup_state(stat_list[-50])
        self.ppid = int(stat_list[-49])
        #self.pgrp = int(stat_list[-48])
        #self.session = int(stat_list[-47])
        #self.tty_nr = int(stat_list[-46])
        #self.tpgid = int(stat_list[-45])
        #self.flags = int(stat_list[-44])
        #self.minflt = int(stat_list[-43])
        #self.cminflt = int(stat_list[-42])
        #self.majflt = int(stat_list[-41])
        #self.cmajflt = int(stat_list[-40])
        #self.utime = int(stat_list[-39])
        #self.stime = int(stat_list[-38])
        #self.cutime = int(stat_list[-37])
        #self.cstime = int(stat_list[-36])
        #self.priority = int(stat_list[-35])
        #self.nice = int(stat_list[-34])
        #self.num_threads = int(stat_list[-33])
        #self.itrealvalue = int(stat_list[-32])
        #self.starttime = int(stat_list[-31])
        #self.vsize = int(stat_list[-30])
        #self.rss = int(stat_list[-29])
        #self.rsslim = int(stat_list[-28])
        #self.startcode = int(stat_list[-27])
        #self.endcode = int(stat_list[-26])
        #self.startstack = int(stat_list[-25])
        #self.kstkesp = int(stat_list[-24])
        #self.kstkeip = int(stat_list[-23])
        #self.signal = int(stat_list[-22])
        #self.blocked = int(stat_list[-21])
        #self.sigignore = int(stat_list[-20])
        #self.sigcatch = int(stat_list[-19])
        #self.wchan = int(stat_list[-18])
        #self.nswap = int(stat_list[-17])
        #self.cnswap = int(stat_list[-16])
        #self.exit_signal = int(stat_list[-15])
        #self.processor = int(stat_list[-14])
        #self.rt_priority = int(stat_list[-13])
        #self.policy = int(stat_list[-12])
        #self.delayacct_blkio_ticks = int(stat_list[-11])
        #self.guest_time = int(stat_list[-10])
        #self.cguest_time = int(stat_list[-9])
        #self.start_data = int(stat_list[-8])
        #self.end_data = int(stat_list[-7])
        #self.start_brk = int(stat_list[-6])
        #self.arg_start = int(stat_list[-5])
        #self.arg_end = int(stat_list[-4])
        #self.env_start = int(stat_list[-3])
        #self.env_end = int(stat_list[-2])
        #self.exit_code = int(stat_list[-1])

    def get_args(self):
        if self._args is not None:
            return self._args
        try:
            with open(f'/proc/{self.pid}/cmdline', 'rb') as f:
                args_raw = f.read().decode(errors="replace")
            self._args = args_raw.split('\0')
            return self._args
        except FileNotFoundError:
            return None

    def get_process_str(self):
        if self._process_str is not None:
            return self._process_str

        def escape_arg(arg: str):
            if any(map(lambda x: x.isspace(), arg)) or "'" in arg or '"' in arg:
                if '"' in arg:
                    arg = "'" + arg + "'"
                else:
                    arg = '"' + arg + '"'
            return arg
        args = self.get_args()
        if args is not None and len(args) > 0:
            self._process_str = " ".join(map(escape_arg, args))
            return self._process_str
        else:
            return f"<{self.pid}: cannot read args>"

    def get_ros_name(self):
        args = self.get_args()
        if args is None or len(args) == 0:
            return f"<{self.pid}: cannnot read args>"

        if args[0].endswith("/python3") or args[0].endswith("/python"):
            # For python ros nodes, print the script rather than just python3
            exe_path = args[1]
        else:
            exe_path = args[0]

        return f"{os.path.basename(exe_path)} ({self.pid})"

def get_service_cgroup():
    with open('/proc/self/cgroup') as f:
        groups = f.readlines()
        if len(groups) > 1:
            raise RuntimeError("Multiple cgroups found (cgroupv1 system?):\n\t" + "\t".join(groups))
        elif len(groups) == 0:
            raise RuntimeError("Process is not part of a cgroup")
        group = groups[0].strip()
        hid, controllers, path = group.split(":")
        if hid != "0" or controllers != "":
            raise RuntimeError("Invalid cgroup v2 definition (maybe v1 system?)")
        path = path.strip('/') # Remove both leading and trailing slash for service detection and os.path.join
        if not path.endswith('.service'):
            raise RuntimeError(f"Process is not in a service top level cgroup! (found {path})\n"
                               "Make sure you run this program as a service in --systemd mode!")
        sysfs_path = os.path.join("/sys/fs/cgroup", path)
        if not os.path.exists(sysfs_path):
            raise RuntimeError(f"Could not find cgroup info in expected directory: {sysfs_path}")
        return sysfs_path

def get_cgroup_processes(cgroup_path):
    with open(os.path.join(cgroup_path, "cgroup.procs")) as f:
        strpid_list = f.readlines()
    cgroup_processes: 'dict[int, LinuxProcess]' = {}
    self_pid = os.getpid()
    parent_pid = os.getppid()
    for pid_str in strpid_list:
        pid = int(pid_str.strip())
        if pid == self_pid or pid == parent_pid:
            continue  # Don't add our own process or the pid of the ros2 run command
        try:
            with open(f"/proc/{pid}/stat") as f:
                lines = f.readlines()
                if len(lines) != 1:
                    raise RuntimeError(f"PID {pid} invalid line count: {len(lines)}")
                stat_str = lines[0].strip()
                cgroup_processes[pid] = LinuxProcess(pid, stat_str)
        except FileNotFoundError:
            print(f"Failed to fetch pid after reported alive: {pid}! (must have just died)... Ignoring")
            continue

    # Need to consolidate all children processes under their launches
    direct_children = list(cgroup_processes.values())
    for child in cgroup_processes.values():
        if child.ppid == 1 or child.ppid == self_pid:
            # Direct descendant of this process (or parent died)
            # Not going to be consolidated
            continue
        if child.ppid in cgroup_processes:
            # We have it in the cgroup process, remove the child from direct_children
            parent = cgroup_processes[child.ppid]
            parent.children.append(child)
            direct_children.remove(child)

    # Return dictionary of PIDs to direct children to speed up removal of active launches
    return dict(map(lambda x: (x.pid, x), direct_children))

def dump_children(children: 'list[LinuxProcess]', indent=0):
    for child in children:
        #print(f'{" " * indent} - {child.pid}: {child.get_process_str()} ({child.state})')
        print(f'{" " * indent} - {child.get_ros_name()}')
        dump_children(child.children, indent+4)


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
        self.monitor_procinfo: 'LinuxProcess | None' = None
        self.launch_procinfo: 'LinuxProcess | None' = None

    def _clear_monitored_topics(self):
        for topic in self.monitored_topics:
            topic.seen = False

    def refresh_state(self, process_map: 'dict[int, LinuxProcess]'):
        if self._launch_subproc is None and self._monitor_subproc is None:
            return  # Nothing to do, both processes have been cleaned up

        # Extract the process info from the process map if provided
        if self._launch_subproc is not None and process_map is not None and self._launch_subproc.pid in process_map:
            self.launch_procinfo = process_map.pop(self._launch_subproc.pid)
        else:
            self.launch_procinfo = None

        if self._monitor_subproc is not None and process_map is not None and self._monitor_subproc.pid in process_map:
            self.monitor_procinfo = process_map.pop(self._monitor_subproc.pid)
        else:
            self.monitor_procinfo = None

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
            "is_zombie": o.is_zombie,
            "launch_procinfo": o.launch_procinfo,
            "monitor_procinfo": o.monitor_procinfo
        }
    elif isinstance(o, LaunchData.LaunchTopic):
        return {
            "name": o.name,
            "type": o.type,
            "is_sensor_data": o.is_sensor_data,
            "seen": o.seen
        }
    elif isinstance(o, LinuxProcess):
        return {
            "pid": o.pid,
            "ppid": o.ppid,
            "state": o.state,
            "ros_name": o.get_ros_name(),
            "full_name": o.get_process_str(),
            "children": o.children
        }
    else:
        raise TypeError(f'Object of type {o.__class__.__name__} '
                        f'is not JSON serializable')

class ParentServer:
    def __init__(self, systemd_mode: bool) -> None:
        self.launches: 'list[LaunchData]' = []
        self.current_file = ""
        self.systemd_mode = systemd_mode

        if self.systemd_mode:
            self.cgroup = get_service_cgroup()

    def load_file(self, file_path: str):
        if file_path == self.current_file:
            return

        # Mark all launches as zombies and ask them to terminate if running
        for launch in self.launches:
            launch.is_zombie = True
            launch.refresh_state(None)
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

                elif req_path == "/restart_service":
                    self.send_response(200)
                    self.send_header("Content-type", "text/html")
                    self.end_headers()

                    exit(0)

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
                    # Fetch current process map so we can check for orphans if in systemd mode
                    if parent_self.systemd_mode:
                        process_map = get_cgroup_processes(parent_self.cgroup)
                    else:
                        process_map = None

                    # Refresh the state of all the launch instances
                    for launch_inst in parent_self.launches:
                        launch_inst.refresh_state(process_map)

                    # Clean up any zombies
                    parent_self._cleanup_dead_zombies()

                    status_resp = {
                        "systemd_mode": parent_self.systemd_mode,
                        "launches": parent_self.launches,
                        "orphans": list(process_map.values()) if parent_self.systemd_mode else None
                    }

                    self.send_response(200)
                    self.send_header("Access-Control-Allow-Origin", "*")
                    self.send_header("Content-type", "application/json")
                    self.end_headers()
                    self.wfile.write(json.dumps(status_resp, default=json_encode_default).encode())

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
            launch.refresh_state(None)
            if launch.can_terminate:
                launch.term_launch()

        # Keep looping until all launches are dead
        while len(parent_self.launches) > 0:
            for launch in parent_self.launches:
                launch.refresh_state(None)
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
        if len(sys.argv) > 1 and "--systemd" in sys.argv:
            systemd_mode = True
        else:
            systemd_mode = False
        server = ParentServer(systemd_mode)
        server.do_main()



if __name__ == '__main__':
    main()
