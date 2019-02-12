#ifndef ACTION_TOOLS_HPP
#define ACTION_TOOLS_HPP

#include "ai_msgs/ActionStatus.h"

#include "action_manager/AtomicAction.hpp"
#include "action_manager/ActionBlock.hpp"

#include <memory>

class ActionChoice {
public:
	ActionChoice(std::shared_ptr<AtomicAction> action = nullptr, double score = 0);

	std::shared_ptr<AtomicAction> action;
	double score;
};

namespace ActionTools {
	ActionChoice getOptimalNextAtomic(ActionPtr action, Point& robotPos);
}
#endif