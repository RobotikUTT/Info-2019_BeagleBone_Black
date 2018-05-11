#include "scheduler/ActionManager.h"
#include "ros/ros.h"

using namespace rapidjson;

ActionManager::ActionManager(const char* actions_file){
  this->actionsInit(actions_file);
}

ActionManager::ActionManager(){
}

void ActionManager::actionsInit (const char* actions_file){
  FILE* fp = fopen(actions_file, "r");

  char readBuffer[1000];
  FileReadStream is(fp, readBuffer, sizeof(readBuffer));

  Document doc;
  doc.ParseStream(is);
  fclose(fp);

  const Value& a = doc["actions"];

  for (SizeType i = 0; i < a.Size(); i++){
    int val_x, val_y, val_angle;
    std::vector<int> param;
    if(a[i]["PAction"].HasMember("end")){
      val_x = a[i]["PAction"]["end"]["x"].GetInt();
      val_y = a[i]["PAction"]["end"]["y"].GetInt();
      if(a[i]["PAction"]["end"].HasMember("angle")){
        val_angle = a[i]["PAction"]["end"]["angle"].GetInt();
      } else {
        val_angle = 0;
      }
    } else {
      val_x = -1;
      val_y = -1;
      val_angle = 0;
    }
    int angle;
    if(a[i]["PAction"]["start"].HasMember("angle")){
      angle = a[i]["PAction"]["start"]["angle"].GetInt();
    } else {
      angle = 0;
    }

    if(a[i].HasMember("param")){
      for (SizeType j = 0; j < a[i]["param"].Size(); j++){
        param.push_back(a[i]["param"][j].GetInt());
      }
    }
    ActionPoint tempPoint ( a[i]["PAction"]["start"]["x"].GetInt(),
                            a[i]["PAction"]["start"]["y"].GetInt(),
                            angle,
                            val_x,
                            val_y,
                            val_angle);


    // std::cout << tempPoint << '\n';
    ActionClass temp (a[i]["name"].GetString(),
                      a[i]["action"].GetInt(),
                      tempPoint,
                      a[i]["point"].GetInt(),
                      a[i]["difficulty"].GetFloat(),
                      param);
    // std::cout << temp << '\n';

    this->action.push_back(temp);
  }
}

void ActionManager::changeSide(){
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
          res.point.Opoint.x    = it->PAction.startPoint.x;
          res.point.Opoint.y    = it->PAction.startPoint.y;
          res.point.Opoint.rot  = it->PAction.startPoint.angle;
          res.point.timeout     = *it->_param.begin();
        } else if(it->_action == BLOCK) {
          res.action_pos.x      = it->PAction.startPoint.x;
          res.action_pos.y      = it->PAction.startPoint.y;
          res.action_pos.rot    = it->PAction.startPoint.angle;
          res.depot_pos.x       = it->PAction.endPoint.x;
          res.depot_pos.y       = it->PAction.endPoint.y;
          res.depot_pos.rot     = it->PAction.endPoint.angle;
        }
        else if (it->_action == BALL) {
          res.action_pos.x      = it->PAction.startPoint.x;
          res.action_pos.y      = it->PAction.startPoint.y;
          res.action_pos.rot    = it->PAction.startPoint.angle;
          res.depot_pos.x       = it->PAction.endPoint.x;
          res.depot_pos.y       = it->PAction.endPoint.y;
          res.depot_pos.rot     = it->PAction.endPoint.angle;
          res.param.clear();
          std::vector<int>::iterator i;
          for(i = it->_param.begin(); i != it->_param.end(); ++i){
            res.param.push_back(*i);
          }
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
  // ROS_INFO_STREAM("Action done: " << done);
  current_action->_done = done;
}
