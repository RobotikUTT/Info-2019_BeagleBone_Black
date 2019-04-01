Value Argumentable::get(string name) {
	map::const_iterator pos = map.find(name);

	if (pos == map.end()) {
		throw "not found";
	} else {
		return pos->second;
	}
}

long Argumentable::getLong(string name, long defaultValue = 0) {
	if (!this->has(name)) {
		return defaultValue;
	}

	Value arg = this->get(name, args);

	if (arg.type != Value::INT_TYPE) {
		throw "invalid argument type";
	}

	return arg.intValue;
}

double Argumentable::getDouble(string name, double defaultValue = 0) {
	if (!this->has(name)) {
		return defaultValue;
	}
	
	Value arg = this->get(name, args);

	if (arg.type != Value::FLOAT_TYPE) {
		throw "invalid argument type";
	}

	return arg.floatValue;

}

string Argumentable::getString(string name, string defaultValue = "") {
	if (!this->has(name)) {
		return defaultValue;
	}

	Argument arg = this->get(name, args);

	if (arg.type != Argument::STRING_TYPE) {
		throw "invalid argument type";
	}

	return arg.stringValue;
}

void Argumentable::setLong(string name, long value);
void Argumentable::setDouble(string name, double value);
void Argumentable::setString(string name, string value);

bool Argumentable::has(string name);
bool Argumentable::hasLong(string name);
bool Argumentable::hasFloat(string name);
bool Argumentable::hasString(string name);

void fromList(vector<Argument>);

long Argumentable::getLong(string name, long defaultValue /*= 0*/, vector<Argument>* args /*= NULL*/) {
	
}

double Argumentable::getDouble(string name, double defaultValue /*= 0*/, vector<Argument>* args /*= NULL*/) {
}

string Argumentable::getString(string name, string defaultValue /*= ""*/, vector<Argument>* args = /*NULL*/) {
}

/**
 *  Retrieve an argument from the argument list
 */
Value Argumentable::get(string name, vector<Argument>* args /*= NULL*/) {
	// No arg list provided -> use current action's one
	if (args == NULL) {
		args = &this->_args;
	}

	// Seek an argument with this name
	for (const auto& next : *args) {
		if (name.compare(next.name) == 0) {
			return next;
		}
	}

	return defaultValue;
}

bool Argumentable::hasArg(string name, vector<Argument>* args /*= NULL*/) {
	// No arg list provided -> use current action's one
	if (args == NULL) {
		args = &_args;
	}

	// Seek an argument with this name
	for (const auto& next : *args) {
		if (name.compare(next.name) == 0) {
			return true;
		}
	}

	return false;
}