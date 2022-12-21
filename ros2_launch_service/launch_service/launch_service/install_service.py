# the goal of this script is to create, install and configure service units for auto starting our code
from argparse import ArgumentParser
import shutil
from glob import glob
from subprocess import Popen, PIPE, CalledProcessError, call
import os.path, os

# template ROS Service
SVC_PATTERN = '''
[Unit]
Description=Service to bring up a ros network 
After=network.target

[Service]
Type=simple
ExecStart=/bin/bash -ic "ros2 run {0} {1}"
StandardOutput=inherit
StandardError=inherit
Restart={2}
User={3}
Group={3}

[Install]
WantedBy={4}.target
'''

def execute(fullCmd, printOut=False):
    if printOut: print(fullCmd)
    proc = Popen(fullCmd, stdout=PIPE, stderr=PIPE, universal_newlines=True)
    if printOut:
        for line in iter(proc.stdout.readline, ""):
            print(line)
        for errLine in iter(proc.stderr.readline, ""):
            print(f"ERROR: {errLine}")
    proc.stdout.close()
    retCode = proc.wait()
    if retCode:
        raise CalledProcessError(retCode, fullCmd)

def main():
    parser = ArgumentParser(description="Setup a systemd service unit file")
    # always needed args
    parser.add_argument("name", type=str, help="name of the service to create")
    # parser.add_argument("ros_launch_package", type=str, help="name of the package with the launch file inside")
    # parser.add_argument("ros_executable", type=str, help="name of the executable to start")

    # optional args
    parser.add_argument("-r" "--restart", type=str, help="restart policy to use", default='on-failure', required=False)
    parser.add_argument("-u", "--username", type=str, help="Username to login with", default=os.getlogin(), required=False)
    parser.add_argument("-t", "--target", type=str, help="systemd target to join", default='multi-user', required=False)
    args = parser.parse_args()

    SVC_NAME = args.name
    SVC_ROS_PKG = 'launch_service' # args.ros_launch_package
    SVC_LAUNCH =  'ros_launcher' # args.ros_executable
    SVC_RESTART = args.r__restart
    SVC_USER = args.username
    SVC_TARGET = args.target

    print(f"Creating ros2 launch service {SVC_NAME}")
    print(f"Service will launch {SVC_LAUNCH} from {SVC_ROS_PKG}")
    print(f"Restart policy is {SVC_RESTART}")
    print(f"Running with user {SVC_USER}")

    serviceFile = SVC_PATTERN
    serviceFile = serviceFile.format(SVC_ROS_PKG, SVC_LAUNCH, SVC_RESTART, SVC_USER, SVC_TARGET)
    serviceFileName = f"ros2_{SVC_NAME}.service"

    tmpFile = os.path.join("/tmp", serviceFileName)
    print(f"Writing to service file {tmpFile}")
    with open(tmpFile, 'w') as file:
        file.write(serviceFile)

    try:
        # move the source file into the correct location
        systemFile = os.path.join("/etc", "systemd", "system", serviceFileName)
        print(f"Moving service file to {systemFile}")
        execute(["sudo", "mv", tmpFile, systemFile], True)

        # register, enable and start the service
        execute(['sudo', 'systemctl', 'daemon-reload'], True)
        execute(['sudo', 'systemctl', 'enable', serviceFileName], True)
        # execute(['sudo', 'systemctl', 'start', serviceFileName], True)

    except CalledProcessError as e:
        print("Error occured while configuring SystemD")
        print(e)

if __name__ == "__main__":
    main()