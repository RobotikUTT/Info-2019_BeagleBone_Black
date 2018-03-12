#ifndef STATE_H
#define STATE_H

class State
{
public:
	uint8_t ROBOT_INIT     = 0; // nodes initializing
	uint8_t ROBOT_READY    = 1; // all nodes ready, waiting start
	uint8_t ROBOT_RUNNING  = 2; // robot in game
	uint8_t ROBOT_HALT     = 3; // end of game
	
};

#endif