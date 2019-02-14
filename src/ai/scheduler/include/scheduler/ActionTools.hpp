#ifndef ACTION_TOOLS_HPP
#define ACTION_TOOLS_HPP

#include "ai_msgs/ActionStatus.h"

#include "action_manager/AtomicAction.hpp"
#include "action_manager/ActionBlock.hpp"

#include <memory>

class ActionChoice {
public:
	ActionChoice(AtomicActionPtr action = nullptr, double score = 0);

	AtomicActionPtr action;
	double score;
};

ActionChoice getOptimalNextAtomic(ActionPtr action, Point& robotPos);

#endif