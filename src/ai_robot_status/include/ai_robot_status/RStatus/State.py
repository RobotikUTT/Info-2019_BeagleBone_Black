class RobotState(object):
	"""docstring for status
	class for set robot status"""
	ROBOT_INIT     = 0 # nodes initializing
	ROBOT_READY    = 1 # all nodes ready, waiting start
	ROBOT_RUNNING  = 2 # robot in game
	ROBOT_HALT     = 3 # end of game


class WatcherState(object):
	"""docstring for status
	class for set watcher status"""

	NODES_INIT    	= 0 # nodes initializing
	NODES_RUNNING 	= 1 # nodes running
	NODES_ERROR 	= 2 # at least 1 node error

	PIN_ON			= 0 # Game hold
	PIN_OFF			= 1 # Game start

	NODES_CHECKLIST = {

	# "/namespace/pkg" 	: None,
	"/sender/" 			: None, #test
	"/receiver/" 		: None  #test

	}