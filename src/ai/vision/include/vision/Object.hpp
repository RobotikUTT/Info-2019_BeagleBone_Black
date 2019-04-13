#ifndef OBJECT_HPP
#define OBJECT_HPP

#include "action_manager/Argumentable.hpp"
#include "shape.hpp"

#include <string>
#include <vector>

using std::string;
using std::vector;

class ObjectAction {

};

class Object : public Argumentable {
public:
	string name;
	Shape shape;
	vector<ObjectAction> actions;
	map<string, Argumentable> argsSets;

	Object();
	Object(const Object& parent);
};

class Zone : public Argumentable {
public:
	Shape shape;
	string name;
};

#endif