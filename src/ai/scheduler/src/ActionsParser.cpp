
#include "scheduler/ActionsParser.hpp"
#include "ros/ros.h"

using namespace rapidjson;


/**
 * Parse constructor
 */
ActionsParser::ActionsParser(){ }


Action ActionsParser::parse(const char* filename) {
    std::list<Action> actions;

    // opening file
    FILE* fileP = fopen(filename, "r");

    char readBuffer[1000];
    FileReadStream readStream(fileP, readBuffer, sizeof(readBuffer));

    // reading document
    Document document;
    document.ParseStream(readStream);

    Value object = document.GetObject();

    return parseAction(object);
}

Action ActionsParser::parseAction(Value object) {
    // if not specified, an action is atomic
    if (object["composed"] != true) {
        return parseAtomicAction(object);
    }

    // TODO : loop over elements of "actions" and add to a list, then create a group action, take "ordered" into consideration 
    /*for () {

    }*/
    throw "todo";
}

Action ActionsParser::parseAtomicAction(Value object) {
    // TODO
    throw "todo";
}
