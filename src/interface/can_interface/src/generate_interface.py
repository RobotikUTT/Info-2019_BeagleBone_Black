#!/usr/bin/python3
# Generate C++ code to bridge interface_msgs to ROS frames
# TODO

from xml.etree import ElementTree
from importlib import import_module
import sys

from ioelements import InputElement, OutputElement, InterfaceCode

# Output file name
if len(sys.argv) < 2:
	print("please specify output file")
	exit(0)

output_file = sys.argv[-1]

# Elements to be generated
elements = []

# Parsing mapping file
root = ElementTree.parse("mapping.xml").getroot()

if root.tag != "mapping":
	print("invalid file, must contains a <mapping> root element")
	exit(0)

for child in root:
	if child.tag == "input":
		elements.append(InputElement(child.attrib, child))
	elif child.tag == "output":
		elements.append(OutputElement(child.attrib, child))
	else:
		print("unknow element :", child.tag)

# Generating content
code = InterfaceCode()

for element in elements:
	element.generate(code)

print(code)