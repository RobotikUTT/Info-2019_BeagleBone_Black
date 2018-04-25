#include "scheduler/ActionManager.h"
#include "ros/ros.h"

using namespace rapidjson;

ActionManager::ActionManager(const char* actions_file){
  this->actionsInit(actions_file);
}

ActionManager::ActionManager(){
}


void ActionManager::actionsInit (const char* actions_file){
  // char* ACTIONS_FILE;
  // nh.param<char*>("~config_file", ACTIONS_FILE, "action_manager/actions.config");
  FILE* fp = fopen(actions_file, "r");

  char readBuffer[1000];
  FileReadStream is(fp, readBuffer, sizeof(readBuffer));

  Document doc;
  doc.ParseStream(is);
  fclose(fp);

  const Value& a = doc["actions"];

  for (SizeType i = 0; i < a.Size(); i++){

    int val_x, val_y;
    if(a[i]["PAction"].HasMember("end")){
      val_x = a[i]["PAction"]["end"]["x"].GetInt();
      val_y = a[i]["PAction"]["end"]["y"].GetInt();
    } else {
      val_x = -1;
      val_y = -1;
    }

    ActionPoint tempPoint ( a[i]["PAction"]["start"]["x"].GetInt(),
                            a[i]["PAction"]["start"]["y"].GetInt(),
                            val_x,
                            val_y);


    // std::cout << tempPoint << '\n';
    ActionClass temp (a[i]["name"].GetString(),
                      a[i]["action"].GetInt(),
                      tempPoint,
                      a[i]["point"].GetInt(),
                      a[i]["difficulty"].GetFloat());
    // std::cout << temp << '\n';

    this->action.push_back(temp);

  }
}


void ActionManager::changeSide() {
  std::list<ActionClass>::iterator it;
  for(it = action.begin(); it != action.end(); ++it){
    it->changeSide();
  }
}

void ActionManager::updatePriority(Point robot_pos){
  std::list<ActionClass>::iterator it;
  for(it = action.begin(); it != action.end(); ++it){
    it->updatePriority( robot_pos );
  }
}

void ActionManager::getActionToDo(ai_msgs::GetActionToDo::Response &res){
  std::string action_name;
  res.action_val = -1;
  int min_prio = std::numeric_limits<int>::max();
  std::list<ActionClass>::iterator it;
  for(it = action.begin(); it != action.end(); ++it){
    if (!it->_done) {
      if (min_prio > it->_priority) {
        res.action_val = it->_action;
        action_name = it->_name;
        if (it->_action == MOVE) {
          res.point.end_x = it->PAction.startPoint.x;
          res.point.end_y = it->PAction.startPoint.y;
        }
        min_prio = it->_priority;
        current_action = it;
      }
    }
  }
  // return action_val;
  ROS_WARN_STREAM("action name: " << action_name);
}


void ActionManager::currentActionDone(bool done){
  ROS_INFO_STREAM("Action done: " << done);
  current_action->_done = done;
}
