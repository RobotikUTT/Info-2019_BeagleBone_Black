#ifndef ACTION_TOOLS_HPP
#define ACTION_TOOLS_HPP

#include "action_manager/Action.hpp"
#include "action_manager/AtomicAction.hpp"
#include "action_manager/ActionBlock.hpp"

#include <memory>

class ActionChoice {
public:
	ActionChoice(std::shared_ptr<AtomicAction> action = nullptr, double score = 0);

	std::shared_ptr<AtomicAction> action;
	double score;
};

/**
 * Compute optimal atomic action to run from now, based on time of
 * realisation and points it give.
 * 
 * This function respect the sync parameter and therefore return the
 * right action to do.
 */
ActionChoice getOptimalNextAtomic(ActionPtr action, Point& robotPos);

/**
 * Remove actions from paused state
 * @return whether some actions were previously paused and now resumed
 */
bool unpauseActions(ActionPtr action); // TODO

#endif