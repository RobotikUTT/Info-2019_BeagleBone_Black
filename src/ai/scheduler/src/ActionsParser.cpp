#include "scheduler/ActionsParser.hpp"

using namespace rapidjson;


ActionsParser::ActionsParser(const char* filepath) : actionRoot(parseFile(filepath)) {}

Action ActionsParser::getAction() {
    return actionRoot;
}

FILE* ActionsParser::openFile(const char* filepath) {
    // check that the path hasn't been explored
    for(const auto& next : filesExplored) {
        if (strcmp(next, filepath) == 0) {
            throw "circular JSON inclusion";
        }
    }

    // append path to the list
    filesExplored.push_back(filepath);

    // finaly open the file
    FILE* fileP = fopen(filepath, "r");

    if (fileP == NULL) {
        std::stringstream message;
        message << "unable to locate" << filepath;
        throw message.str();
    }

    return fileP;
}

Action ActionsParser::parseFile(const char* filename) {
    FILE* fileP = openFile(filename);

    char readBuffer[1000];
    FileReadStream readStream(fileP, readBuffer, sizeof(readBuffer));

    // reading document
    Document document;
    document.ParseStream(readStream);

    const Value& object = document;

    // begin to parse
    return parseAction(object);
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