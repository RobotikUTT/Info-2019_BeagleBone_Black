# Message fields
uint8 robot_status
uint8 error_code

# Message constants for robot_status
uint8 ROBOT_INIT = 0 # nodes initializing
uint8 ROBOT_READY = 1 # all nodes ready, waiting start
uint8 ROBOT_RUNNING = 2 # robot in game
uint8 ROBOT_HALT = 3