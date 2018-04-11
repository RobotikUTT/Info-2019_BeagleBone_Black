#include "scheduler/ActionPoint.h"


ActionPoint::ActionPoint(){
  this->startPoint = Point(0,0);
  this->endPoint = Point(0,0);
}

ActionPoint::ActionPoint(Point start, Point end){
  this->startPoint = start;
  this->endPoint = end;
}

ActionPoint::ActionPoint(int start_x, int start_y, int end_x, int end_y){
  this->startPoint = Point(start_x, start_y);
  this->endPoint = Point(end_x, end_y);;
}


std::ostream& operator<<(std::ostream& os, const ActionPoint& AP)
{
    os <<  "ActionPoint : { \n\t start_" << AP.startPoint << "\n\t end_" << AP.endPoint << "\n}";
    return os;
}
