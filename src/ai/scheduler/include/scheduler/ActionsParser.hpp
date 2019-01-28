#ifndef ACTION_PARSER_NODE_H
#define ACTION_PARSER_NODE_H

#include "scheduler/AtomicAction.hpp"
#include "scheduler/ComposedAction.hpp"

#include "rapidjson/document.h"
#include "rapidjson/filereadstream.h"

#include <list>
#include <vector>
#include <iostream>
#include <string>
#include <limits>

/**
 * @brief      Class for action manager.
 */
class ActionsParser
{
public:
  ActionsParser();

  Action parse(const char*);

//private:
  Action parseAction(rapidjson::Value object);
  Action parseAtomicAction(rapidjson::Value object);
};

#endif
