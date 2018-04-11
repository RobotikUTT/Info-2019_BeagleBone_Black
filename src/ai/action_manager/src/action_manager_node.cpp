#include "action_manager/action_manager_node.h"

using namespace rapidjson;

ActionManager::ActionManager(ros::NodeHandle *n){

  this->nh = *n;

  this->side = SIDE_GREEN;

  this->side_sub = nh.subscribe("ai/side", 1, &ActionManager::setSide, this);

  this->actionsInit();

  service_ready("ai", "action_manager", 1 );

}


void ActionManager::actionsInit (){
  // char* ACTIONS_FILE;
  // nh.param<char*>("~config_file", ACTIONS_FILE, "action_manager/actions.config");
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
  if(msg->side != this->side){
    this->side = !this->side;
    std::list<ActionClass>::iterator it;
    for(it = action.begin(); it != action.end(); ++it){
      it->changeSide();
    }
  }
}

int main(int argc, char *argv[]) {
  ros::init(argc,argv, "action_manager_node");

	ros::NodeHandle nmh;

  ActionManager node (&nmh);

	ros::spin();
}
