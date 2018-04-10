#include "action_manager_node.h"
#include "robot_watcher/SetSide.h"

using namespace rapidjson;

ActionManager::ActionManager(){

  this->side_sub = nh.subscribe("ai/side", 1, &ActionManager::setSide, this);

  this->actionsInit();

}


void ActionManager::actionsInit (){
  FILE* fp = fopen(ACTIONS_FILE, "r");

  char readBuffer[65536];
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
                      tempPoint,
                      a[i]["point"].GetInt(),
                      a[i]["difficulty"].GetFloat());
    // std::cout << temp << '\n';

    this->action.push_back(temp);

  }
}


void ActionManager::setSide(const robot_watcher::SetSide::ConstPtr& msg) {
  if(msg->side){
    std::list<ActionClass>::iterator it;
    for(it = v.begin(); it != v.end(); ++it){
      it->changeSide();
    }

  }
}
