from pathlib import Path
import re

class InterfaceCode:
	def __init__(self, source_folder):
		self.public_header = []
		self.private_header = []
		self.reception_switch_cases = []
		self.constructor_content = []
		self.includes = []

		self.source_file = source_folder + "template.cpp"

	def add_switch_case(self, frame_value, code):
		self.reception_switch_cases.append(
			"case Frame::{}: {{\n{}\n}}\nbreak;\n".format(frame_value, code)
		)

	def __str__(self):
		contents = Path(self.source_file).read_text()
	
		contents = contents.format(**{
			"public_header": "\n".join(self.public_header),
			"private_header": "\n".join(self.private_header),
			"reception_switch_cases": "\n".join(self.reception_switch_cases),
			"includes": "\n".join(map(lambda x: "#include \"{0}\"".format(x), list(dict.fromkeys(self.includes)))),
			"constructor_content": "\n".join(self.constructor_content)
		})

		return '''
		/*
		 * THIS FILE IS GENERATED AUTOMATICALLY
		 * Do not try to edit this file manually unless
		 * you know what you are doing.
		 *
		 * If you want to see how the content is generated, please
		 * take a look at [can_interface]/src/generate_interface.py
		 */
		'''.replace("\t", "") + contents