#ifndef ARGUMENTABLE_HPP
#define ARGUMENTABLE_HPP

#include "ai_msgs/Argument.h"
#include "ai_msgs/Value.h"

#include <vector>
#include <string>

using ai_msgs::Value;
using ai_msgs::Argument;

class Argumentable {
public:
	Value get(string name);

	long getLong(string name, long defaultValue = 0, vector<Argument>* args = NULL);
	double getDouble(string name, double defaultValue = 0, vector<Argument>* args = NULL);
	string getString(string name, string defaultValue = "", vector<Argument>* args = NULL);
	
	void setLong(string name, long value);
	void setDouble(string name, double value);
	void setString(string name, string value);

	bool has(string name);
	bool hasLong(string name);
	bool hasFloat(string name);
	bool hasString(string name);

	void fromList(vector<Argument>);
private:
	map<string, Value> values;
};

#endif