#include "scheduler/ActionsParser.hpp"

using namespace rapidjson;

// TODO add arguments for files imported actions (to have generic actions for example)
ActionsParser::ActionsParser(std::string filepath) : actionRoot(parseFile(filepath)) { }

Action ActionsParser::getAction() {
    return actionRoot;
}

Action ActionsParser::parseFile(std::string filepath) {
    // check that the path hasn't been explored
    for(const auto& next : filesExplored) {
        if (next == filepath) {
            throw "circular JSON inclusion";
        }
    }

    // append path to the list
    filesExplored.push_back(filepath);

    try {
        // finaly open the file
        std::ifstream filestream(filepath);
        IStreamWrapper wrapper(filestream);

        Document d;
        d.ParseStream(wrapper);

        const Value& object = d;

        // begin to parse
        return parseAction(object);
    } catch(std::ios_base::failure& e) {
        throw e.what();
    }
}

Action ActionsParser::parseAction(const Value& object) {
    // objects define atomic action
    if (object.IsObject()) {
        // If the action has to be loaded from a different file
        if (object.HasMember("file") && object["file"].IsString()) {
            // Load from file
            return parseFile(object["file"].GetString());
        }

        return parseAtomicAction(object);
    }
    
    // while arrays define composed action
    else if (object.IsArray()) {
        std::list<Action> actions;
        AtomicAction* descriptor;

        for (Value::ConstValueIterator itr = object.Begin(); itr != object.End(); ++itr) {
            // first object must define the block
            if (itr == object.Begin()) {
                if (!itr->IsObject()) {
                    throw "invalid block descriptor";
                }
                AtomicAction desc = parseAtomicAction(*itr, true);
                descriptor = &desc;
            } else {
                actions.push_back(parseAction(*itr));
            }
        }

        return ActionBlock(*descriptor, actions);
    }

    throw "invalid object";
}

AtomicAction ActionsParser::parseAtomicAction(const Value& object, const bool allowNoPerfomer /* = false*/) {
    // an action must be named
    if (!object.HasMember("name") || !object["name"].IsString()) {
        throw "missing or invalid name";
    }

    // and most of times a perfomer
    if (!allowNoPerfomer && (!object.HasMember("performer") || !object["performer"].IsString())) {
        throw "missing or invalid action perfomer";
    }

    std::string name = object["name"].GetString();
    std::string performer = allowNoPerfomer ? "" : object["performer"].GetString();

    AtomicAction action(name, performer);

    // parse optional content
    if (object.HasMember("sync") && object["sync"].IsBool()) {
        action.setSync(true);
    }

    if (object.HasMember("points") && object["points"].IsInt()) {
        action.setBasePoints(object["points"].GetInt());
    }

    if (object.HasMember("args") && object["sync"].IsObject()) {
        parseArgs(object["sync"], action);
    }

    return action;
}

void ActionsParser::parseArgs(const Value& object, AtomicAction& targetAction) {
    ai_msgs::Argument* arg;

    for (Value::ConstMemberIterator itr = object.MemberBegin(); itr != object.MemberEnd(); ++itr) {
        arg = new ai_msgs::Argument();
        arg->name = itr->name.GetString();
        arg->value = itr->value.GetDouble();

        targetAction.addArg(*arg);
    }
}