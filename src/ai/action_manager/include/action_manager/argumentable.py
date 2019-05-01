from ai_msgs.msg import Argument

from typing import List, Dict, Union

from xml_class_parser import Parsable, Bind, Slice, BindDict

# Make argument msg parsable
Argument = Parsable(name=Bind(to="name"), content=Bind(to="value"))(Argument)

@Parsable(
	name = Bind(to="name"),
	children=[ # Parse arguments 
		BindDict(to="values", type=Argument, key="name", post_cast=Slice("value"))
	]
)
class Argumentable:
	"""
		This object make it easier to manipulate variant type argument.
	"""
	def __init__(self):
		self.name = ""
		self.values: Dict[str, str] = {}
		
	def get(self, name: str, type: type = str, default = None) -> str:
		if name in self.values:
			return type(self.values[name])
		else:
			return default
		
	def set(self, name: str, value: Union[str, int, float]) -> None:
		"""
			Set value to attr with given value
		"""
		self.values[name] = value.__str__()

	def has(self, name: str, type: type):
		if name in self.values:
			try:
				type(self.values[name])
			except ValueError:
				return False
			return True
		return False


	def from_list(self, args: List[Argument], reset: bool = False) -> 'Argumentable':
		if reset:
			self.values = {}

		for arg in args:
			self.values[arg.name] = arg.value
		
		return self
	
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