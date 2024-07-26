# Python 3 server example
import asyncio
import contextlib
import http
import logging
import os, sys, json, yaml, signal
import enum
import mimetypes
import platform
import re
import time
import signal
import uuid
import websockets
import weakref

from ament_index_python.packages import get_package_share_directory

SUPER_SECRET_LAUNCH_FLAG = "--super-secret-forking-flag"
SUPER_SECRET_MONITOR_FLAG = "--super-secret-monitoring-flag"

log_format = "[%(name)s] %(message)s"
log_level = logging.INFO
serverPort = 8080

HTML_SERVER_ROOT = os.path.join(get_package_share_directory("remote_launch"), "pages")

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

        if args[0].endswith("/python3") or args[0].endswith("/python") and len(args) > 1:
            # For python ros nodes, use the script name (1st arg) rather than just python3
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
            print(f"Failed to fetch pid after reported alive: {pid}! (must have just died)... Ignoring", file=sys.stderr)
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

class LaunchState(enum.Enum):
    STOPPED = "stopped"
    STARTING = "starting"
    RUNNING = "running"
    STOPPING = "stopping"
    STOPPING_ERROR = "stopping_error"
    ERROR = "error"

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

    def __init__(self, launch: dict, parent_logger: 'logging.Logger', broadcast_refresh_event: 'asyncio.Event'):
        self.output_websockets = set()

        # Launch Information (decoded from JSON)
        self.id = str(uuid.uuid4())
        self.friendly_name: str = launch["file"].removesuffix(".launch.py")
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
        self.logger = parent_logger.getChild(self.friendly_name)

        # Process Tracking
        self._monitor_subproc = None
        self._launch_subproc = None
        self.monitor_procinfo: 'LinuxProcess | None' = None
        self.launch_procinfo: 'LinuxProcess | None' = None

        # Asyncio Control
        self._launch_task_inst: 'asyncio.Task | None' = None
        self._stop_launch_event: 'asyncio.Event | None' = None
        self._broadcast_refresh_event = broadcast_refresh_event  # Notifies the server that the status has changed

    def _clear_monitored_topics(self):
        for topic in self.monitored_topics:
            topic.seen = False

    async def _monitor_monitor_task(self):
        # Monitors the launch monitor task
        while True:
            # Handle the byte indices coming in
            monitor_out = await self._monitor_subproc.stdout.read(1)
            if len(monitor_out) == 0:
                # Stdout must have closed (process probably stopped)
                break

            topic_idx = monitor_out[0]
            if topic_idx < len(self.monitored_topics):
                self.monitored_topics[topic_idx].seen = True
                self._broadcast_refresh_event.set()
            else:
                self.logger.warning(f"Unexpected index received on stdin: {topic_idx} ('{chr(topic_idx)}')")

        # Wait for process to fully terminate
        await self._monitor_subproc.wait()

    async def _launch_monitor_task(self):
        while True:
            line_encoded = await self._launch_subproc.stdout.readline()
            line = line_encoded.decode(errors="replace")
            if len(line) == 0:
                # We reached EOF, just wait for launch to terminate
                break
            if len(self.output_websockets):
                state = self.state
                if state == LaunchState.RUNNING:
                    # Fix up state for starting/running so we can see the difference between launch starting/running
                    if self._monitor_subproc is not None:
                        all_topics_seen = True
                        for topic in self.monitored_topics:
                            if not topic.seen:
                                all_topics_seen = False
                                break
                    else:
                        all_topics_seen = True

                    if not all_topics_seen:
                        state = LaunchState.STARTING

                log_obj = {
                    "type": "log",
                    "time": time.time(),
                    "name": self.friendly_name,
                    "state": state,
                    "data": line.rstrip('\n')
                }
                websockets.broadcast(self.output_websockets, json.dumps(log_obj, default=json_encode_default))

        await self._launch_subproc.wait()

    async def _launch_task(self):
        # Asyncio task to manage the launching/monitoring/shutdown of the given launch

        # launch state already set to starting in the function that calls this task
        # No need to send refresh event though since the starting phase should only happen briefly
        # The processes should start almost immediately, then we can set to running and broadcast the event

        self.logger.info("Launch Starting")

        # launch fork
        popen_args = ["/proc/self/exe", __file__, SUPER_SECRET_LAUNCH_FLAG,
                      self.package, self.file]
        popen_args.extend(self.args)
        self._launch_subproc = await asyncio.create_subprocess_exec(*popen_args, stdout=asyncio.subprocess.PIPE,
                                                                    stderr=asyncio.subprocess.STDOUT)

        # pipe stdout to this process with noblock
        if len(self.monitored_topics) > 0:
            popen_args = ["/proc/self/exe", __file__, SUPER_SECRET_MONITOR_FLAG]
            popen_args.extend(self._gen_monitor_cmd_args())
            self._monitor_subproc = await asyncio.create_subprocess_exec(*popen_args, stdout=asyncio.subprocess.PIPE)
        else:
            self._monitor_subproc = None

        # Update the launch internal state
        self._clear_monitored_topics()
        self.state = LaunchState.RUNNING

        # Processes running, notify the update
        self._broadcast_refresh_event.set()

        # Create all monitor tasks and events to stop the launch
        self._stop_launch_event = asyncio.Event()
        stop_launch_wait_task = asyncio.create_task(self._stop_launch_event.wait(), name=self.id + "_stop_launcH_wait")
        if self._monitor_subproc is not None:
            monitor_mon_task = asyncio.create_task(self._monitor_monitor_task(), name=self.id + "_monitor_mon")
        else:
            monitor_mon_task = None
        launch_mon_task = asyncio.create_task(self._launch_monitor_task(), name=self.id + "_launch_mon")

        wait_group = {launch_mon_task, stop_launch_wait_task}
        if monitor_mon_task is not None:
            wait_group.add(monitor_mon_task)

        while self.state == LaunchState.RUNNING:
            # Wait for any of the tasks to finish
            done, _ = await asyncio.wait(wait_group, return_when=asyncio.FIRST_COMPLETED)

            if stop_launch_wait_task in done:
                # Launch stop requested
                # Cleanup the successful event
                stop_launch_wait_task.result()  # Grab the result (to buble up any exceptions)
                wait_group.remove(stop_launch_wait_task)
                stop_launch_wait_task = None
                self._stop_launch_event = None

                self.logger.info("Launch Stopping...")
                self.state = LaunchState.STOPPING
                break

            if monitor_mon_task in done:
                # Monitor terminated
                # Clean asyncio tasks and subprocess
                wait_group.remove(monitor_mon_task)
                monitor_mon_task.result()  # Grab the result (to bubble up any exceptions)
                monitor_mon_task = None
                assert self._monitor_subproc.returncode is not None, "Monitor mon task returned before subproc waited!"
                self._monitor_subproc = None

                # Make sure we received all the topics, if not the monitor must have crashed, and we should error
                all_topics_seen = True
                for topic in self.monitored_topics:
                    if not topic.seen:
                        all_topics_seen = False

                if not all_topics_seen:
                    self.logger.error("Launch Monitor died before all topics seen")
                    self.state = LaunchState.STOPPING_ERROR
                    break

            if launch_mon_task in done:
                # Clean asyncio tasks and subprocess
                wait_group.remove(launch_mon_task)
                launch_mon_task.result()  # Grab the result (to bubble up any exceptions)
                launch_mon_task = None
                assert self._launch_subproc.returncode is not None, "Launch mon task returned before subproc waited!"
                self._launch_subproc = None

                # child has died T_T
                self.logger.error("Launch died")

                # Launch must have crashed, stop the launch in an error state
                self.state = LaunchState.STOPPING_ERROR
                break


        # Stopping or stopping in error
        # Wait for all remaining processes
        if self._launch_subproc is not None:
            try:
                self._launch_subproc.send_signal(signal.SIGINT)
            except ProcessLookupError:
                # It's okay if we fail, that means the process died just as we sent the signal
                # However, we want to set STOPPING_ERROR so we know it died rather than potentially stopping gracefully
                # if we stopped due to the stop_launch event
                self.state = LaunchState.STOPPING_ERROR

        if self._monitor_subproc is not None:
            try:
                self._monitor_subproc.send_signal(signal.SIGINT)
            except ProcessLookupError:
                # Same as above, but if monitor dies then it isn't as bad, just keep the last reported state
                pass

        self._broadcast_refresh_event.set()

        # Cancel the stop launch wait if it didn't fire
        if stop_launch_wait_task is not None:
            stop_launch_wait_task.cancel()
            stop_launch_wait_task = None
            self._stop_launch_event = None

        # Wait for all monitor tasks (& thus processes) to terminate
        if launch_mon_task is not None:
            await launch_mon_task
            launch_mon_task = None
            assert self._launch_subproc.returncode is not None, "Launch mon task returned before subproc waited!"
            self._launch_subproc = None

        if monitor_mon_task is not None:
            await monitor_mon_task
            monitor_mon_task = None
            assert self._monitor_subproc.returncode is not None, "Monitor mon task returned before subproc waited!"
            self._monitor_subproc = None

        # Cleanup monitored topics and set final state for launch
        self._clear_monitored_topics()
        if self.state == LaunchState.STOPPING:
            self.state = LaunchState.STOPPED
        else:
            self.state = LaunchState.ERROR
        self._broadcast_refresh_event.set()

        self.logger.info(f"Launch Cleaned Up")


    def update_process_map(self, process_map: 'dict[int, LinuxProcess]'):
        # Extract the process info from the process map if provided
        if self._launch_subproc is not None and process_map is not None and self._launch_subproc.pid in process_map:
            self.launch_procinfo = process_map.pop(self._launch_subproc.pid)
        else:
            self.launch_procinfo = None

        if self._monitor_subproc is not None and process_map is not None and self._monitor_subproc.pid in process_map:
            self.monitor_procinfo = process_map.pop(self._monitor_subproc.pid)
        else:
            self.monitor_procinfo = None


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

    @property
    def is_dead_zombie(self) -> bool:
        # Fetch the launch task result and free task if its done but hasn't been cleared yet
        # This property *should* be read somewhere in the background so any exceptions will be quickly bubbled up
        if self._launch_task_inst is not None and self._launch_task_inst.done():
            # Get the result of the future so any exceptions thrown will bubble up to caller
            self._launch_task_inst.result()
            self._launch_task_inst = None

        am_dead = self.is_zombie and self._launch_task_inst is None
        if am_dead and (self._launch_subproc is not None or self._monitor_subproc is not None):
            raise RuntimeError("Launch task terminated without cleaning up processes!")
        return am_dead

    def perform_launch(self):
        if self._launch_task_inst is not None:
            if not self._launch_task_inst.done():
                raise RuntimeError("Attempting to start a launch when launch already in progress")
            else:
                # Fetch result of task before clearing it so any exceptions can bubble up
                self._launch_task_inst.result()

        self.state = LaunchState.STARTING
        self._launch_task_inst = asyncio.create_task(self._launch_task(), name=self.id + "_launch_task")

    def term_launch(self):
        if self._stop_launch_event is None:
            raise RuntimeError("Attempting to kill an already cleaned up launch")

        self._stop_launch_event.set()

def json_encode_default(o):
    if isinstance(o, LaunchState):
        return o.value
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

async def event_wait(evt, timeout):
    # suppress TimeoutError because we'll return False in case of timeout
    with contextlib.suppress(asyncio.TimeoutError):
        await asyncio.wait_for(evt.wait(), timeout)
    return evt.is_set()

class LaunchServer:
    class WebsocketHandler(logging.Handler):
        def __init__(self, connections: set)-> None:
            self._connections_weak = weakref.ref(connections)
            logging.Handler.__init__(self=self)

        def emit(self, record) -> None:
            connections = self._connections_weak()
            if connections is None or len(connections) == 0:
                return

            log_obj = {
                "type": "log",
                "time": time.time(),
                "name": record.name,
                "state": "global",
                "data": record.getMessage()
            }
            websockets.broadcast(connections, json.dumps(log_obj, default=json_encode_default))

    def __init__(self, systemd_mode: bool) -> None:
        self.launches: 'list[LaunchData]' = []
        self.last_process_map: 'dict[int, LinuxProcess] | None' = None
        self.current_file = ""
        self.systemd_mode = systemd_mode
        self.connections = set()
        self.default_log_connections = set()  # List of all connections to enable logs for a launch by default

        # Logging Config
        self.logger = logging.getLogger("remote_launch")
        self.logger.setLevel(log_level)
        self.logger.addHandler(self.WebsocketHandler(self.connections))

        # Load all launch entries
        pack_path = get_package_share_directory("remote_launch")
        launch_entries = os.listdir(os.path.join(pack_path, "launches"))
        self.launch_files = [x for x in launch_entries if x.endswith('.yaml') or x.endswith('.yml')]

        if self.systemd_mode:
            self.cgroup = get_service_cgroup()

    ########################################
    # Convenience Methods
    ########################################

    @property
    def server_status(self) -> dict:
        return {
            "type": "status",
            "current_launchfile": os.path.basename(self.current_file),
            "launches": self.launches,
            "orphans": list(self.last_process_map.values()) if self.systemd_mode and self.last_process_map is not None else None
        }

    def load_file(self, file_path: str):
        if file_path == self.current_file:
            return

        # Mark all launches as zombies and ask them to terminate if running
        for launch in self.launches:
            launch.is_zombie = True
            if launch.can_terminate:
                launch.term_launch()
        # Purge any dead ones
        self._cleanup_dead_zombies()

        # Populate with the new launch data
        self.current_file = file_path
        with open(file_path, 'r') as file:
            launch_data = yaml.safe_load(file)["launches"]
            for launch in launch_data:
                self.launches.append(LaunchData(launch, self.logger, self.broadcast_refresh_event))
                self.launches[-1].output_websockets = self.default_log_connections.copy()

    def _cleanup_dead_zombies(self):
        self.launches[:] = [inst for inst in self.launches if not inst.is_dead_zombie]

    def _generate_err(self, msg):
        return {"type": "error", "msg": msg}

    ########################################
    # Request Handling/Background Tick Code
    ########################################

    def handle_ws_connect_msg(self, request: dict, websocket: object):
        if request["cmd"] != "connect":
            raise RuntimeError("Invalid connect msg")

        # Default log enrollment
        enable_default_logging = request["logging_enable_default"]
        if enable_default_logging:
            self.default_log_connections.add(websocket)
            for launch in self.launches:
                launch.output_websockets.add(websocket)

        last_launchfile = request["last_launchfile"] if "last_launchfile" in request else None
        if last_launchfile is not None and self.current_file == "" and len(self.launches) == 0:
            # If we are given a last launchfile and we don't have a launch selected, auto select that one
            # This makes loading right on startup quicker so you don't have to select the launch
            pack_path = get_package_share_directory("remote_launch")
            launch_filename = last_launchfile.replace('/', '')  # Path sanitization
            launch_yaml = os.path.join(pack_path, "launches", launch_filename)

            # If the file exists, we're good to select it
            if os.path.isfile(launch_yaml):
                self.logger.info(f"Auto-selecting last used launch definition '{launch_filename}'")
                self.load_file(launch_yaml)
                self.broadcast_refresh_event.set()

    def handle_ws_msg(self, request: dict, websocket: object) -> 'dict | None':
        # Stops the server, gracefully cleaning up launches
        if request["cmd"] == "stop_server":
            self.stop.set()

        # Restarts server immediately without cleaning up launches
        # Can only be used in systemd mode since this WILL leave orphans
        # But, once the service dies systemd will clean everything up for us anyways
        elif request["cmd"] == "kill_server_now":
            if self.systemd_mode:
                os._exit(0)
            else:
                return self._generate_err("Can only perform immediate restart when running in systemd mode")

        # this route allows spawning the launch
        elif request["cmd"] == "start_launch":
            try:
                launch_inst = next(x for x in self.launches if x.id == request["id"])
            except StopIteration:
                return self._generate_err("Invalid Launch ID")

            if launch_inst.can_launch:
                launch_inst.perform_launch()
                self.broadcast_refresh_event.set()
            else:
                return self._generate_err("Cannot Launch Right Now")

        # this route allows stopping a launch
        elif request["cmd"] == "stop_launch":
            try:
                launch_inst = next(x for x in self.launches if x.id == request["id"])
            except StopIteration:
                return self._generate_err("Invalid Launch ID")

            if launch_inst.can_terminate:
                launch_inst.term_launch()
                self.broadcast_refresh_event.set()
            else:
                return self._generate_err("Cannot Stop Right Now")

        # this route allows stopping a launch
        elif request["cmd"] == "load_launch":
            pack_path = get_package_share_directory("remote_launch")
            launch_filename = request["file"].replace('/', '')  # Path sanitization
            launch_yaml = os.path.join(pack_path, "launches", launch_filename)

            self.logger.info(f"Loading launch definition '{launch_filename}'")

            if os.path.isfile(launch_yaml):
                self.load_file(launch_yaml)
                self.broadcast_refresh_event.set()
            else:
                return self._generate_err("Invalid Launch File")

        elif request["cmd"] == "set_logdefault":
            # Adds/removes websocket to all current and future websocket logs

            enable_default = request["enable"]

            # Add/remove the websocket from the global enrollment set
            if enable_default:
                self.default_log_connections.add(websocket)
            elif websocket in self.default_log_connections:
                self.default_log_connections.remove(websocket)

            # Add/remove the websocket from all enrolled launches
            if enable_default:
                for launch in self.launches:
                    launch.output_websockets.add(websocket)
            else:
                for launch in self.launches:
                    if websocket in launch.output_websockets:
                        launch.output_websockets.remove(websocket)

            # Return back list of all launches that this websocket is now enrolled in
            return self.get_logging_enrollment(websocket)

        elif request["cmd"] == "set_logenable":
            # Manually enables/disables a specific launch's log

            enable = request["enable"]
            try:
                launch_inst = next(x for x in self.launches if x.id == request["id"])
            except StopIteration:
                return self._generate_err("Invalid Launch ID")

            if enable:
                launch_inst.output_websockets.add(websocket)
            elif websocket in launch_inst.output_websockets:
                launch_inst.output_websockets.remove(websocket)

            # Report the new logging enrollment
            return self.get_logging_enrollment(websocket)


    async def status_broadcast_task(self):
        try:
            # This runs as a server in the background, and is responsible for broadcasting any updated states to the clients
            while True:
                # Wait for event, or refresh every 1.5 seconds otherwise
                self.broadcast_refresh_event.clear()
                await event_wait(self.broadcast_refresh_event, 1.5)

                # Fetch current process map so we can check for orphans if in systemd mode
                if self.systemd_mode:
                    self.last_process_map = get_cgroup_processes(self.cgroup)
                else:
                    self.last_process_map = None

                # Update the process map in all the launch instances
                for launch_inst in self.launches:
                    launch_inst.update_process_map(self.last_process_map)

                # Clean up any zombies
                self._cleanup_dead_zombies()

                # Broadcast the new server status
                websockets.broadcast(self.connections, json.dumps(self.server_status, default=json_encode_default))
        finally:
            # We crashed, set the stop to tell the server to stop and retreive our exception
            self.stop.set()

    def get_logging_enrollment(self, websocket):
        launch_enrollment_status = {}
        for launch in self.launches:
            launch_enrollment_status[launch.id] = websocket in launch.output_websockets
        return {
            "type": "log_enrollment",
            "launches": launch_enrollment_status,
            "default": websocket in self.default_log_connections
        }

    def get_first_connect_resp(self, websocket) -> dict:
        return {
            "type": "first_connect",
            "launch_files": self.launch_files,
            "systemd_mode": self.systemd_mode,
            "status": self.server_status,
            "log_enrollment": self.get_logging_enrollment(websocket)
        }


    ########################################
    # Underlying Websocket Control Code
    ########################################

    async def process_request(self, req_path: str, request_headers):
        # This function is called whenever a web request is received
        # If None is returned, it'll open a websocket, but if a 3 tuple is returned, it'll send back that
        # status, headers, and response without opening a websocket

        path = req_path.split('?')[0]
        if path == "/ws":
            # Access websocket path, return None so it'll do a websocket
            return None

        def serve_file(filename, status=http.HTTPStatus.OK):
            with open(filename, "rb") as f:
                return status, [("Content-Type", mimetypes.guess_type(filename)[0])], f.read()

        # Path sanitization
        # - pretending to chroot to the current directory
        # - cancelling all redundant paths (/.. = /)
        # - making the path relative
        sanitized_path = os.path.relpath(os.path.normpath(os.path.join("/", path)), "/")
        file_path = os.path.join(HTML_SERVER_ROOT, sanitized_path)
        if os.path.isfile(file_path):
            return serve_file(file_path)
        elif os.path.isdir(file_path):
            index_file = os.path.join(file_path, "index.html")
            if os.path.isfile(index_file):
                return serve_file(index_file)
            else:
                return serve_file(os.path.join(HTML_SERVER_ROOT, "403.html"), http.HTTPStatus.FORBIDDEN)

        # If we got here, return 404 not found
        return serve_file(os.path.join(HTML_SERVER_ROOT, "404.html"), http.HTTPStatus.NOT_FOUND)

    async def ws_conn_handler(self, websocket: 'websockets.WebSocketServerProtocol'):
        # This function is called whenever a new websocket connection is opened

        # First perform initial connection sequence
        try:
            message = await websocket.recv()
            if websocket.closed or message is None:
                return
        except:
            # Silence any incomplete read errors
            return
        self.handle_ws_connect_msg(json.loads(message), websocket)
        await websocket.send(json.dumps(self.get_first_connect_resp(websocket), default=json_encode_default))

        # Then put socket into standard running mode
        self.connections.add(websocket)
        try:
            async for message in websocket:
                resp = self.handle_ws_msg(json.loads(message), websocket)
                if resp is not None:
                    await websocket.send(json.dumps(resp, default=json_encode_default))
        finally:
            self.connections.remove(websocket)

    def _keyboard_int_handler(self, signum, frame):
        # Only set if stop hasn't been set yet
        if self.stop.is_set():
            return

        def stopset():
            self.stop.set()

        self.logger.warning("Keyboard Interrupt!")
        asyncio.get_running_loop().call_soon_threadsafe(stopset)

    async def do_main(self):
        loop = asyncio.get_running_loop()
        self.stop = asyncio.Event()
        signal.signal(signal.SIGINT, self._keyboard_int_handler)

        broadcast_task = loop.create_task(self.status_broadcast_task(), name="Status Broadcast Task")
        self.broadcast_refresh_event = asyncio.Event()
        try:
            async with websockets.serve(self.ws_conn_handler, "0.0.0.0", serverPort,
                                        process_request=self.process_request):
                self.logger.info("Server started http://%s:%s" % (platform.node(), serverPort))
                await self.stop.wait()

            # Check if broadcast task is completed (that means it has crashed)
            # If so, get the result and bubble up the exception
            if broadcast_task.done():
                broadcast_task.result()
        finally:
            signal.signal(signal.SIGINT, signal.SIG_IGN)
            if not broadcast_task.done():
                broadcast_task.cancel()
            await self.cleanup_launches()

    async def cleanup_launches(self):
        # Don't let control c kill it so we don't leave stragglers
        # If you REALLY want to kill it, use `ps f` and `kill` to make sure everything is killed

        # Mark all child launches to die
        for launch in self.launches:
            launch.is_zombie = True
            if launch.can_terminate:
                launch.term_launch()

        # Keep looping until all launches are dead
        self._cleanup_dead_zombies()
        if len(self.launches) > 0:
            self.logger.info("Cleaning up remaining child launches: " + ",".join(x.friendly_name for x in self.launches))
            while len(self.launches) > 0:
                await event_wait(self.broadcast_refresh_event, 1.0)
                self.broadcast_refresh_event.clear()
                self._cleanup_dead_zombies()

        self.logger.info("Gracefully Terminated (All Child Launches Cleaned Up)")


########################################
# Auto-forking Management
########################################

# monitor process stuff
def do_child_monitor():
    # KEEP ALL ROS STUFF OUT OF PARENT PROCESS
    # Keeping ROS nodes as the network is brought up and down is a recipe for obscure bugs in ros to pop up
    # This will ensure that ALL ros nodes are brought down when ROS is brought down
    import rclpy
    from rclpy.node import Node

    logger = logging.getLogger(f"Launch Monitor-{os.getpid()}")

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

    class ChildMonitor(Node):
        def __init__(self, node_name: str, *, context = None, cli_args: 'list[str]' = None, namespace: str = None, use_global_arguments: bool = True, enable_rosout: bool = True, start_parameter_services: bool = True, parameter_overrides = None, allow_undeclared_parameters: bool = False, automatically_declare_parameters_from_overrides: bool = False) -> None:
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

                logger.info(f"Monitoring {topics[i][0]} of type {topics[i][1]} with QOS {topic_qos}")

                # make the subscription
                sub_cb = ChildMonitorCallback(self, i)

                from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default

                sub = None
                exec(f"from {topic_type[0]}.{topic_type[1]} import {topic_type[2]}", locals())
                exec(f"sub = self.create_subscription({topic_type[2]}, '{topic_name}', sub_cb.callback, {topic_qos})", locals())

                self.monitor_subs.append((sub, sub_cb))

    rclpy.init()

    # chack to make sure we have correct args
    if len(sys.argv[2:]) % 3 != 0 or len(sys.argv) <= 2:
        logger.fatal("Malformed launch args passed to monitor child!")
        exit(1)

    # build the topic info to send to the monitor node
    topics = []
    for i in range(2, len(sys.argv), 3):
        topics.append((sys.argv[i], sys.argv[i+1], sys.argv[i+2]))

    # prepare the monitor
    monitor = ChildMonitor(f"monitor_{uuid.uuid4().hex}")
    monitor.start_monitors(topics)

    # free run the monitor until shutdown
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        logger.warning("Interrupted before all topics seen")
    rclpy.try_shutdown()


# flag handlers and launch integration
def main():
    # Setup python logging (using basic format below and logging to stderr)
    logging.basicConfig(format=log_format)

    # Handle the launch startup
    if len(sys.argv) > 2 and sys.argv[1] == SUPER_SECRET_LAUNCH_FLAG:
        os.setpgrp()  # We only want control C to go to the remote launch server

        logger = logging.getLogger(f"Child Launch-{os.getpid()}")
        logger.setLevel(log_level)

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

        logger.info(f"args: {args}")

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
        server = LaunchServer(systemd_mode)
        asyncio.run(server.do_main())


if __name__ == '__main__':
    main()
