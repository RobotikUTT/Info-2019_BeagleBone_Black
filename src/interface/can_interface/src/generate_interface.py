#!/usr/bin/python3
# Generate C++ code to bridge interface_msgs to ROS frames
from xml.etree import ElementTree
from importlib import import_module
import sys

from IOElements import InputElement, OutputElement
from InterfaceCode import InterfaceCode

# Output file name
if len(sys.argv) < 3:
	print("please specify input directory and output file\nusage : ./generate_interface [mapping and template folder] [output file]")
	exit(0)

output_file = sys.argv[-1]
source_folder = sys.argv[-2]

if not source_folder.endswith("/"):
	source_folder = source_folder + "/"

# Elements to be generated
elements = []

# Parsing mapping file
root = ElementTree.parse(source_folder + "mapping.xml").getroot()

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
code = InterfaceCode(source_folder)

for element in elements:
	element.generate(code)

f = open(output_file, "w")
f.write(code.__str__())
f.close()