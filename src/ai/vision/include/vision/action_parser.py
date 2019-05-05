from .parser import ElementParser, AttrParser, Parser

from action_manager import Action, ActionGroup
from args_lib.msg import Argument



def group_type_validator(value: str) -> str:
	available = ["ordered", "unordered", "best"]

	if value in available:
		return value
	elif value in [e[0] for e in available]:
		# Retrieve from list
		return available[[e[0] for e in available].index(value)]

class ActionParser(Parser):
	def __init__(self):
		super().__init__()

		# Action
		self.action_parser = ElementParser(Action, "do", "subactions") \
			.bind_attr("xml", None) \
			.bind_attr("performer", None) \
			.bind_attr("points",  0, shorthand=False, cast=int) \
			.bind_attr("repeat", 1)

		# Actions's argument
		self.argument_parser = ElementParser(Argument, "argument", "arguments", True) \
			.bind_attr("name", "", mandatory=True) \
			.bind_attr("default", "")
		#ArgumentParser.content = ElementParserAttribute("value")

		# Object requirement
		self.object_requirement_parser = ElementParser(dict, "need", "required") \
			.bind_attr("name", "", True) \
			.bind_attr("alias", None) \
			.bind_attr("min", 1, shorthand=False, cast=int) \
			.bind_attr("max", 1, shorthand=False, cast=int) \
			.bind_attr("quantity", 1, cast=int)

		# Action group
		self.group_parser = ElementParser(ActionGroup, "group", "subactions") \
			.bind_attr("type", "ordered", cast=group_type_validator) \
			.bind_attr("repeat", 1)