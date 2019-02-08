/** @file ActionClass.cpp
*	@brief Class for ActionClass.
*		
*		
*	@author Alexis CARE
*/
#include "scheduler/ActionManager.hpp"


/**
 * @brief Constructs the object.
 *
 * @param[in] actions_file The JSON actions file
 */
ActionManager::ActionManager(ActionPtr rootAction){
	
}

/**
 * @brief Change all ActionClass side
 */
void ActionManager::changeSide(){
	std::list<ActionClass>::iterator it;
	for(it = action.begin(); it != action.end(); ++it){
		it->changeSide();
	}
}

/**
 * @brief			Update the done's attribut current Action
 *
 * @param[in]	done	The done
 */
void ActionManager::currentActionDone(bool done){
	// ROS_INFO_STREAM("Action done: " << done);
	current_action->_done = done;
}
