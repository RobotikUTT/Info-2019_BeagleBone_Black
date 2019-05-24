import os
import rospkg

modules = []
module_name = "input_simulation"

# Import all modules in include directory
for module in os.listdir("{}/include/{}/modules".format(rospkg.RosPack().get_path(module_name), module_name)):
	if module == '__init__.py' or module[-3:] != '.py':
		continue

	imported = __import__("input_simulation.modules.{}".format(module[:-3]), locals(), globals(), ["register"])
	modules.append(imported)

del module
del module_name