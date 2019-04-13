#include "vision/ObjectParser.hpp"

Object ObjectParser::get(string objectName) {
	auto index = this->objects.find(objectName);

	if (index != this->objects.end()) {

	} else {

	}
}

void ObjectParser::parseObjects(string filepath) {
	pugi::xml_document doc;
	pugi::xml_parse_result res = doc.load_file(filepath.c_str());

	if (res) {
		pugi::xml_node root = doc.child("objects");

		// Parse all children
		for (pugi::xml_node_iterator it = root.begin(); it != root.end(); ++it) {
			Object parsed = this->parseObject(*it);

			this->objects[parsed.name] = parsed;
		}

	} else {
		ROS_ERROR_STREAM("Unable to parse XML");// : " << res.description());
	}
}

Object ObjectParser::parseObject(pugi::xml_node& node) {
	const string extends = node.attribute("extends").as_string();
	ROS_INFO_STREAM(node.name());// << " extends " << extends);
	ROS_INFO_STREAM(extends);// << " extends " << extends);
}

Shape ObjectParser::parseShape() {

}