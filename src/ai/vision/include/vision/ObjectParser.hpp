#ifndef OBJECT_PARSER_HPP
#define OBJECT_PARSER_HPP

#include "pugixml/pugixml.hpp"

#include "vision/Object.hpp"
#include "vision/shape.hpp"

#include <string>
#include <map>

using std::string;
using std::map;

class ObjectParser {
public:
	void parseObjects(string filename);

	Object get(string objectName);
private:
	map<string, Object> objects;

	Object parseObject(pugi::xml_node&);
	Shape parseShape();
	
};

#endif