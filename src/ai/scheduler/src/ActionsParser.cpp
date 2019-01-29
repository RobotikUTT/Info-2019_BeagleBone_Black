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
    }

    // TODO : loop over elements of "actions" and add to a list, then create a group action, take "ordered" into consideration 
    /*for () {

    }*/
    throw "todo";
}

AtomicAction ActionsParser::parseAtomicAction(const Value& object, const bool allowNoPerfomer /* = false*/) {
    
    // TODO
    throw "todo";
}
