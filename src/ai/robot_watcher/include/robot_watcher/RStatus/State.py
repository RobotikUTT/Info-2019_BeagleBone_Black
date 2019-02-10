
## @file RStatus/State.py
##   @brief File to define alias
##   
##   File documentation
##   
##   
##   @author Alexis CARE
##

## 
## @defgroup Robot_watcher The Robot_watcher package
## @{



##@brief      Class for robot state.

class RobotState(object):
	ROBOT_INIT     = 0 # nodes initializing
	ROBOT_READY    = 1 # all nodes ready, waiting start
	ROBOT_RUNNING  = 2 # robot in game
	ROBOT_HALT     = 3 # end of game


##@brief Class for Watcher State

class WatcherState(object):

	NODES_INIT    	= 1 # nodes initializing
	NODES_RUNNING 	= 2 # nodes running
	NODES_ERROR 	= 3 # at least 1 node error

	PIN_ON			= 0 # Game hold
	PIN_OFF			= 1 # Game start

##
## @brief      Class for side.
##
class Side(object):
	GREEN = 0
	ORANGE = 1

NODES_CHECKLIST = {

# "/namespace/pkg" 		: None,
# "/sender/" 				: None, #test
# "/receiver/" 			: None, #test
"/board/STM" 			: "optional",
"/board/ARDUINO" 		: "optional",
"/board/ZIGBEE" 		: "optional",
"/board/PANEL" 			: "optional",
"/ros_can/interface" 	: "need",
"/ai/scheduler" 		: "need",
"/ai/controller" 		: "need",
"/procedure/move" 		: "need"


}

## @}
