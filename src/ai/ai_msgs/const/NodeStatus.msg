# Message field
int8 state_code
int8 error_code

# Constants for state_code
int8 ASKING = -1 # code used to ask for a node readiness
int8 UNKNOW = 0
int8 INIT = 1
int8 READY = 2
int8 ERROR = 3
int8 DESTROYED = 4

