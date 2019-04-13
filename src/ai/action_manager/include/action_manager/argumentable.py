from ai_msgs.msg import Argument, Value

from typing import List, Dict

class Argumentable:
	def __init__(self):
		self.values: Dict[str, Value] = {}
		
	def get(self, name: str) -> Value:
		if name in self.values:
			return self.values[name]
		
		raise Exception("{} does not match with an argument name".format(name))

	def get_int(self, name: str, defaultValue: int = 0) -> int:
		value = self.get(name)

		if value.type != Value.LONG:
			raise Exception("{} does not have int type".format(name))
		else:
			return value.longValue

	def get_float(self, name: str, defaultValue: float = 0) -> float:
		value = self.get(name)

		if value.type != Value.DOUBLE:
			raise Exception("{} does not have float type".format(name))
		else:
			return value.floatValue

	def get_string(self, name: str, defaultValue = "") -> str:
		value = self.get(name)

		if value.type != Value.STRING:
			raise Exception("{} does not have float type".format(name))
		else:
			return value.stringValue
		
	def set_int(self, name: str, value: int) -> None:
		if name not in self.values:
			self.values[name] = Value()

		self.values[name].type = Value.LONG
		self.values[name].longValue = value

	def set_float(self, name: str, value: float) -> None:
		if name not in self.values:
			self.values[name] = Value()

		self.values[name].type = Value.DOUBLE
		self.values[name].floatValue = value

	def set_string(self, name: str, value: str) -> None:
		if name not in self.values:
			self.values[name] = Value()

		self.values[name].type = Value.STRING
		self.values[name].stringValue = value

	def has(self, name: str) -> bool:
		return name in self.values

	def has_int(self, name: str) -> bool:
		return self.has(name) and self.values[name].type == Value.LONG

	def has_float(self, name: str) -> bool:
		return self.has(name) and self.values[name].type == Value.DOUBLE

	def has_string(self, name: str) -> bool:
		return self.has(name) and self.values[name].type == Value.STRING

	def from_list(self, args: List[Argument], reset: bool = False) -> None:
		if reset:
			self.values = {}

		for arg in args:
			self.values[arg.name] = arg.value
	
	def to_list(self) -> List[Argument]:
		result = []

		for key in self.values:
			arg = Argument()
			arg.name = key
			arg.value = self.values[key]

			result.append(arg)

		return result