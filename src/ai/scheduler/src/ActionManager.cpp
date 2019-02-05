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
 * @brief			Update all ActionClass Priority
 *
 * @param[in]	robot_pos	The robot position
 */
void ActionManager::updatePriority(Point robot_pos){
	std::list<ActionClass>::iterator it;
	for(it = action.begin(); it != action.end(); ++it){
		it->updatePriority( robot_pos );
	}
}

/**
 * @brief			Gets the next action to execute
 *
 * @param[in, out]			res	 The service response message
 */
void ActionManager::getActionToDo(ai_msgs::GetActionToDo::Response &res){
	std::string action_name;
	res.action_val = -1;
	int min_prio = std::numeric_limits<int>::max();
	std::list<ActionClass>::iterator it;
	for(it = action.begin(); it != action.end(); ++it){
		if (!it->_done) {
			if (min_prio > it->_priority) {
				res.action_val = it->_action;
				action_name = it->_name;
				if (it->_action == MOVE) {
					res.point.Opoint.x		= it->PAction.startPoint.x;
					res.point.Opoint.y		= it->PAction.startPoint.y;
					res.point.Opoint.rot	= it->PAction.startPoint.angle;
					res.point.timeout		 = *it->_param.begin();
				} else if(it->_action == BLOCK) {
					res.action_pos.x			= it->PAction.startPoint.x;
					res.action_pos.y			= it->PAction.startPoint.y;
					res.action_pos.rot		= it->PAction.startPoint.angle;
					res.depot_pos.x			 = it->PAction.endPoint.x;
					res.depot_pos.y			 = it->PAction.endPoint.y;
					res.depot_pos.rot		 = it->PAction.endPoint.angle;
				}
				else if (it->_action == BALL) {
					res.action_pos.x			= it->PAction.startPoint.x;
					res.action_pos.y			= it->PAction.startPoint.y;
					res.action_pos.rot		= it->PAction.startPoint.angle;
					res.depot_pos.x			 = it->PAction.endPoint.x;
					res.depot_pos.y			 = it->PAction.endPoint.y;
					res.depot_pos.rot		 = it->PAction.endPoint.angle;
					res.param.clear();
					std::vector<int>::iterator i;
					for(i = it->_param.begin(); i != it->_param.end(); ++i){
						res.param.push_back(*i);
					}
				}
				min_prio = it->_priority;
				current_action = it;
			}
		}
	}
	// return action_val;
	ROS_WARN_STREAM("action name: " << action_name);
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
