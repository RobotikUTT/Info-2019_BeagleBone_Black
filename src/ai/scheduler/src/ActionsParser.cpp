#include "scheduler/ActionsParser.hpp"

using namespace rapidjson;

// TODO add arguments for files imported actions (to have generic actions for example)
ActionsParser::ActionsParser(std::string filepath) {
    actionRoot = new ActionBlock("ROOT");

    parseFile(filepath, *actionRoot);
}

ActionPtr ActionsParser::getAction() {
    return actionRoot->subactions().front();
}

void ActionsParser::parseFile(std::string filepath, ActionBlock& container) {
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
        parseAction(object, container);
    } catch(std::ios_base::failure& e) {
        throw e.what();
    }
}

void ActionsParser::parseAction(const Value& object, ActionBlock& container) {
    // objects define atomic action
    if (object.IsObject()) {
        // If the action has to be loaded from a different file
        if (object.HasMember("file") && object["file"].IsString()) {
            // Load from file
            parseFile(object["file"].GetString(), container);
        } else {
            parseAtomicAction(object, container);
        }
    }
    
    // while arrays define composed action
    else if (object.IsArray()) {
        parseActionBlock(object, container);
    }

    else {
        throw "invalid object";
    }
}

void ActionsParser::parseActionBlock(const Value& object, ActionBlock& container) {
    Value::ConstValueIterator itr = object.Begin();
    const Value& descriptor = *itr;

    // first object must define the block
    if (!descriptor.IsObject()) {
        throw "invalid block descriptor";
    }

     if (!descriptor.HasMember("name") || !descriptor["name"].IsString()) {
        throw "missing or invalid name";
    }

    ActionBlock action(std::string(descriptor["name"].GetString()));

    // parse optional content
    if (descriptor.HasMember("sync") && descriptor["sync"].IsBool()) {
        action.setSync(object["sync"].GetBool());
    }

    if (descriptor.HasMember("points") && descriptor["points"].IsInt()) {
        action.setBasePoints(descriptor["points"].GetInt());
    }

    // then all following ones are contained inside
    ++itr;
    for (; itr != object.End(); ++itr) {
        parseAction(*itr, action);
    }
    
    container.addAction(std::make_shared<ActionBlock>(action));
}

void ActionsParser::parseAtomicAction(const Value& object, ActionBlock& container) {
    // an action must be named
    if (!object.HasMember("name") || !object["name"].IsString()) {
        throw "missing or invalid name";
    }

    // and most of times a perfomer
    if (!object.HasMember("performer") || !object["performer"].IsString()) {
        throw "missing or invalid action perfomer";
    }

    std::string name = object["name"].GetString();
    std::string performer = object["performer"].GetString();

    AtomicAction action = AtomicAction(name, performer);

    // parse optional content
    if (object.HasMember("sync") && object["sync"].IsBool()) {
        action.setSync(object["sync"].GetBool());
    }

    if (object.HasMember("points") && object["points"].IsInt()) {
        action.setBasePoints(object["points"].GetInt());
    }

    if (object.HasMember("args") && object["args"].IsObject()) {
        parseArgs(object["args"], action);
    }

    container.addAction(std::make_shared<AtomicAction>(action));
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