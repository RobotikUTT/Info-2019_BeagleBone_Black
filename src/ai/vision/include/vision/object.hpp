#ifndef OBJECT_HPP
#define OBJECT_HPP

#include <string>
#include <vector>

using std::string;
using std::vector;

const int INT_TYPE = 0;
const int DOUBLE_TYPE = 0;
const int STRING_TYPE = 0;

class Argument {
	int intValue;
	double doubleValue;
	string stringValue;
	
	int type;
};



class Object {
public:
	string name;
	Shape shape;
	vector<Argument> args;
	vector<ObjectAction> actions;

	Object();
	Object(Object parent);
};

class Zone {
public:
	Shape shape;
	string name;
};

#endif