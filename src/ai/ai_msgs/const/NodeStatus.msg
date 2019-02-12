# Message field
int8 state_code
int8 error_code

# Constants for state_code
int8 NODE_ASKING = -1 # code used to ask for a node readiness
int8 NODE_UNKNOW = 0
int8 NODE_INIT = 1
int8 NODE_READY = 2
int8 NODE_ERROR = 3
int8 NODE_DESTROYED = 4

