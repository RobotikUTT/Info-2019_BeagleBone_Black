from action_manager import Action, ActionGroup

from geometry_msgs.msg import Pose2D
from ai_msgs.msg import ActionStatus

class ActionChoice:
	def __init__(self, action=None, score = 0):
		self.action = action
		self.score = score

def get_optimal_next_atomic(action: Action, robot_pos: Pose2D):
	"""
		Compute optimal atomic action to run from now, based on time of
		realisation and points it give.

		This function respect the sync parameter and therefore return the
		right action to do.
	"""

	# Filter actions paused or finished
	if action.state != ActionStatus.IDLE:
		return ActionChoice()
	
	# Try to cast as atomic action
	if type(action) == Action:
		# And return it in that case
		return ActionChoice(action, action.priority(robot_pos))
	
	# Otherwise if group
	if isinstance(action, ActionGroup):
		# Null choice at first
		choice = ActionChoice()

		# Try all subactions
		for next in action.children:
			next_choice = get_optimal_next_atomic(next, robot_pos)

			# If the action has a better score
			if next_choice.score > choice.score:
				# We choose it !
				choice = next_choice
			

			# If we hit a unfinished action in an ordered group, it must
			# be performed in priority
			if action.type == "ordered" and next.state != ActionStatus.DONE:
				break
		

		# As this is a group, we use the group's points instead of the
		# atomic action's points
		choice.score = action.priority(robot_pos)
	
		return choice
	
	
	# error: neither action group nor atomic action
	return ActionChoice()

