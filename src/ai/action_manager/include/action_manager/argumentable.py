from ai_msgs.msg import Argument

from typing import List, Dict, Union

class Argumentable:
	"""
		This object make it easier to manipulate variant type argument.
	"""
	def __init__(self):
		self.values: Dict[str, str] = {}
		
	def get(self, name: str) -> str:
		if name in self.values:
			return self.values[name]
		
		raise Exception("{} does not match with an argument name".format(name))

	def get_int(self, name: str, defaultValue: int = 0) -> int:
		return int(self.get(name)) if self.has(name) else defaultValue

	def get_float(self, name: str, defaultValue: float = 0) -> float:
		return float(self.get(name)) if self.has(name) else defaultValue

	def get_string(self, name: str, defaultValue = "") -> str:
		return self.get(name) if self.has(name) else defaultValue
	
	def set(self, name: str, value: Union[str, int, float]) -> None:
		"""
			Set value to attr with given value
		"""
		self.values[name] = value.__str__()

	def has(self, name: str) -> bool:
		return name in self.values

	def is_int(self, name: str) -> bool:
		if self.has(name):
			try:
				int(self.values[name])
			except ValueError:
				return False
			return True
		return False
		

	def has_float(self, name: str) -> bool:
		if self.has(name):
			try:
				float(self.values[name])
			except ValueError:
				return False
			return True
		return False

	def has_string(self, name: str) -> bool:
		return self.has(name)

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
	
	def __str__(self):
		return ", ".join(["{} = {}".format(name, self.values[name]) for name in self.values])