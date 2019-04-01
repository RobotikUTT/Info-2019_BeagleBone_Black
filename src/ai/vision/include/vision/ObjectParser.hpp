#ifndef OBJECT_PARSER_HPP
#define OBJECT_PARSER_HPP

#include "vision/object.hpp"

#include <string>
#include <map>

using std::string;
using std::map;

class ObjectParser {
public:
	ObjectParser(string filename);

	Object get(string objectName);
private:
	map<string, Object> objects;

	Object parseObject();
	
};

#endif