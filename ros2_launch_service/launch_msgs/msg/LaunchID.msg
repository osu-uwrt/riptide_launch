# constant to indicate that this is a fully formed request
uint32 NO_ID = 0

# enumeration for the process status
uint8 UNKNOWN = 0
uint8 QUEUED = 1
uint8 RUNNING = 2
uint8 ERRORED = 3
uint8 KILLED = 4
uint8 EXITED = 5

# the name of the package to fins the launch file in, if launching with CLI format
string package
# the name of the launch file or the full path to the launch file
string launch_file
# the id assigned to the launch file. the number should be selected to be greater than NO_ID
uint32 launch_id
# the status of the launch. Only populated when querying status
uint8 status
