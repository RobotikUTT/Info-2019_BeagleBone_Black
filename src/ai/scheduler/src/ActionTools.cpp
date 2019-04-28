#include "scheduler/ActionTools.hpp"

ActionChoice::ActionChoice(AtomicActionPtr action /*= nullptr*/, double score /*= 0*/)
	: action(action), score(score) {}

/**
 * Compute optimal atomic action to run from now, based on time of
 * realisation and points it give.
 * 
 * This function respect the sync parameter and therefore return the
 * right action to do.
 */
ActionChoice getOptimalNextAtomic(ActionPtr action, Pose2D& robotPos) {
	// Filter actions paused or finished
	if (action->state() != ActionStatus::IDLE) {
		return ActionChoice();
	}
	
	// Try to cast as atomic action
	const auto atomic = std::dynamic_pointer_cast<AtomicAction>(action);
	if (atomic) {
		// And return it in that case
		return ActionChoice(atomic, atomic->priority(robotPos));
	}
	
	// Otherwise cast as ActionBlock
	const auto block = std::dynamic_pointer_cast<ActionBlock>(action);
	if (block) {
		ActionChoice currentChoice = ActionChoice();

		for (const auto& next : block->subactions()) {
			ActionChoice nextChoice = getOptimalNextAtomic(next, robotPos);

			// If the action has a better score
			if (nextChoice.score > currentChoice.score) {
				// We choose it !
				currentChoice = nextChoice;
			}

			// If we hit a unfinished sync action, it has to be performed
			// before anything else, so we break the loop
			if (next->isSync() && next->state() != ActionStatus::DONE) {
				break;
			}
		}

		// As this is a block, we use the block's points instead of the
		// atomic action's points
		currentChoice.score = block->priority(robotPos);
	
		return currentChoice;
	}
	
	// error: neither action block nor atomic action
	return ActionChoice();
}
