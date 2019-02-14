#ifndef ACTION_PARSER_H
#define ACTION_PARSER_H

#include "action_manager/AtomicAction.hpp"
#include "action_manager/ActionBlock.hpp"

#include "ai_msgs/Argument.h"

#include "rapidjson/document.h"
#include "rapidjson/filereadstream.h"

#include "scheduler/ActionFilePath.hpp"

#include <iostream>
#include <list>
#include <fstream>
#include <rapidjson/istreamwrapper.h>

/**
 * @section DESCRIPTION
 * @brief Parser for actions and actions group from JSON format.
 * 
 * Actions are defined by JSON objects describing all it's properties (name, performer...)
 * 
 * Example :
 *  {
 *      name: string,
 *      perfomer*: string,
 *      args?: {arg1: number, arg2: number, arg3: number},
 *      sync?: boolean[false],
 *      points?: int 
 *  }
 * 
 * (* perfomer is not mandatory in case it describe a block)
 * 
 * Sync define whether the action must be completed before the following ones in a
 * action group. Used for actions requiring others
 * 
 * It can also be loaded from an external file (containing an action or action group)
 * 
 * Example:
 *  {
 *      file: string
 *  }
 * 
 * Actions groups are defined as arrays and contains a descriptor which is the
 * first action contained inside. This action define the block's behavior (sync or not)
 * and how many points are done when completed.
 * 
 * Example:
 * 
 *  [
 *      descriptorAction: Action,
 *      action1,
 *      action2,
 *      [
 *          action31,
 *          action32
 *      ]
 *  ]
 * 
 *  A action group is done when all its actions are complete.
 */
class ActionsParser
{
public:
  ActionsParser(ActionFilePath file, ActionBlockPtr container = nullptr, ActionsParser* parent = nullptr);
  
  ActionPtr getAction();
  bool wasExplored(ActionFilePath path);
private:
  ActionsParser* parent;
  ActionBlockPtr actionRoot;
  ActionFilePath filepath;

  void parseAction(const rapidjson::Value& object, ActionBlockPtr container);
  void parseAtomicAction(const rapidjson::Value& object, ActionBlockPtr container);
  void parseActionBlock(const rapidjson::Value& object, ActionBlockPtr container);

  void parseArgs(const rapidjson::Value& object, AtomicAction& targetAction);
};


#endif
