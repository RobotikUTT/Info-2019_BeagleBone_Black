class InterfaceCode:
	def __init__(self):
		self.public_header = []
		self.private_header = []
		self.reception_switch_cases = []
		self.constructor_content = []

	def add_switch_case(self, frame_value, code):
		self.reception_switch_cases.append(
			"case Frame::{}:\n{}\nbreak;".format(frame_value, code)
		)

	def __str__(self):
		for pub_header in self.public_header:
			print(pub_header)

		for pub_header in self.private_header:
			print(pub_header)

		for pub_header in self.reception_switch_cases:
			print(pub_header)

		for pub_header in self.constructor_content:
			print(pub_header)

class Param:
	def __init__(self, name, pos, size=1):
		self.name = name
		self.size = size
		self.pos = pos
		self.byte_start = 0

class IOElement:
	def __init__(self, settings, params):
		self.settings = settings
		self.params = []

		# Convert params XML list to param list
		for param in params:
			self.params.append(Param(
				param.attrib["name"],
				int(param.attrib["pos"]),
				int(param.attrib["size"])
			))

		# Sort params by their relative position
		sorted(self.params, key=lambda p: p.pos)

		# Compute byte_start for each value (offset 0 saved for frame)
		current_offset = 1
		for param in self.params:
			param.byte_start = current_offset
			current_offset += param.size

	def generate(self, code):
		pass

class InputElement(IOElement):
	def __init__(self, settings, params):
		IOElement.__init__(self, settings, params)

	def publisher_name(self):
		return "{}_pub".format(self.settings["topic"].replace("/", "_"))

	# Generate code to retrieve data from frame data with binary
	# operations
	# Example :
	# 1st byte is pos_x and pos_y has 2 byte after that
	# Generated code should be :
	# msg_to_ai.pos_x = msg->data[0]
	# msg_to_ai.pos_y = msg->data[2] | msg->data[1] << 8;
	def get_content_code(self, param):
		value = "msg_from_can->data[{}]".format(param.byte_start + param.size - 1)

		for index in range(param.size - 1):
			value = "{0} | msg_from_can->data[{1}] << {2}".format(
				value,
				param.byte_start + index,
				(param.size - index - 1) * 8
			)

		return "msg_to_ai.{0} = {1};\n".format(param.name, value)

	def generate(self, code):
		codelines = [
			# Message generation
			"interface_msgs::{0} msg_to_ai;\n".format(self.settings["msg"])
		]

		# Add all parameters
		for param in self.params:
			codelines.append(self.get_content_code(param))

		# Message sending
		codelines.append("{}.publish(msg_to_ai);".format(self.publisher_name()))

		# Add to code cases
		code.add_switch_case(self.settings["frame"], "".join(codelines))

		# Add publisher to declarations
		code.private_header.append(
			"ros::Publisher {};".format(self.publisher_name())
		)

		# And to constructor
		code.constructor_content.append(
			"this->{} = nh.advertise<interface_msgs::{}>(\"{}\", 10);"
				.format(self.publisher_name(), self.settings["msg"], self.settings["topic"])
		)

class OutputElement(IOElement):
	def __init__(self, settings, params):
		IOElement.__init__(self, settings, params)

	def subscriber_name(self):
		return "{}_sub".format(self.settings["topic"].replace("/", "_"))

	def function_name(self):
		return "{}_sub".format(self.settings["topic"].replace("/", ""))
	
	# Generate code to set data in frame with binary
	# operations
	def get_content_code(self, param):
		if param.size == 1:
			return "fr.data[{}] = msg->{};\n".format(param.byte_start, param.name)
		elif param.size == 2:
			return "fr.data[{0}] = msg->{2} >> 8;\nfr.data[{1}] = msg->{2} & 0x00FF;\n".format(
				param.byte_start, param.byte_start + 1, param.name
			)
		else:
			print("size not handled yet, go back to coding")
			return ""

	def generate(self, code):
		codelines = [
			"void CanInterfaceNode::{}(const std_msgs::{}::ConstPtr& msg)"
				.format(self.function_name(), self.settings["msg"]),
			''' {				
				can_msgs::Frame fr;
				fr.header.stamp = ros::Time::now();
				fr.header.frame_id = "/ros_can/interface/";
				fr.is_rtr = 0;
				fr.is_error = 0;
				fr.is_extended = 0;

				fr.dlc = 1;
				fr.id = Frame::STM_CAN_ADDR;
			'''.replace("\t", "")
		]

		codelines.append("fr.data[0] = Frame::{};\n".format(self.settings["frame"]))

		# Add all parameters
		for param in self.params:
			codelines.append(self.get_content_code(param))

		# Message sending
		codelines.append("can_pub.publish(fr);\n}\n")

		# Add to code cases
		code.public_header.append("".join(codelines))

		# Add publisher to declarations
		code.private_header.append(
			"ros::Subscriber {};".format(self.subscriber_name())
		)

		# And to constructor
		code.constructor_content.append(
			"this->{} = nh.subscribe(\"{}\", 10, &CanInterfaceNode::{}, this);"
				.format(self.subscriber_name(), self.settings["topic"], self.function_name())
		)
		pass
		