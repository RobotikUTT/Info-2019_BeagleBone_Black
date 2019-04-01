#include "action_manager/Argumentable.hpp"

Value Argumentable::get(string name) const {
	auto pos = this->values.find(name);

	if (pos == this->values.end()) {
		throw "not found";
	} else {
		return pos->second;
	}
}

long Argumentable::getLong(string name, long defaultValue /*= 0*/) const {
	if (!this->has(name)) {
		return defaultValue;
	}

	Value arg = this->get(name);

	if (arg.type != Value::LONG) {
		throw "invalid argument type";
	}

	return arg.longValue;
}

double Argumentable::getDouble(string name, double defaultValue /*= 0*/) const {
	if (!this->has(name)) {
		return defaultValue;
	}
	
	Value arg = this->get(name);

	if (arg.type != Value::DOUBLE) {
		throw "invalid argument type";
	}

	return arg.doubleValue;

}

string Argumentable::getString(string name, string defaultValue /*= ""*/) const {
	if (!this->has(name)) {
		return defaultValue;
	}

	Value arg = this->get(name);

	if (arg.type != Value::STRING) {
		throw "invalid argument type";
	}

	return arg.stringValue;
}

void Argumentable::setLong(string name, long value) {
	Value val;
	val.longValue = value;
	val.type = Value::LONG;
	this->values[name] = val;
}

void Argumentable::setDouble(string name, double value) {
	Value val;
	val.doubleValue = value;
	val.type = Value::DOUBLE;
	this->values[name] = val;
}

void Argumentable::setString(string name, string value) {
	Value val;
	val.stringValue = value;
	val.type = Value::STRING;
	this->values[name] = val;
}

bool Argumentable::has(string name) const {
	auto pos = this->values.find(name);

	return pos != this->values.end();
}

bool Argumentable::hasLong(string name) const {
	return has(name) && get(name).type == Value::LONG;
}

bool Argumentable::hasDouble(string name) const {
	return has(name) && get(name).type == Value::DOUBLE;
}

bool Argumentable::hasString(string name) const {
	return has(name) && get(name).type == Value::STRING;
}

void Argumentable::fromList(vector<Argument> args, bool reset /*= false*/) {
	if (reset) {
		this->values.clear();
	}

	// Fill map with values
	for (const auto& next : args) {
		this->values[next.name] = next.value;
	}
}

vector<Argument> Argumentable::toList() const {
	vector<Argument> list;

	for (auto const& x : this->values) {
		Argument next;
		next.name = x.first;
		next.value = x.second;

		list.push_back(next);
	}

	return list;
}
