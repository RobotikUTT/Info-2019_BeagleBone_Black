#ifndef FAKE_STM_HPP
#define FAKE_STM_HPP

#include <ros/ros.h>

#include "can_msgs/Finish.h"
#include "can_msgs/RobotBlocked.h"
#include "std_msgs/Empty.h"
#include "can_msgs/Speed.h"
#include "can_msgs/Point.h"

using namespace can_msgs;
using ros::Publisher;
using ros::Subscriber;

const int ROTATION = 0;
const int POSITION = 1;
const int ORIENTED_POSITION = 2;

struct Goal {
    Point point;
    int type;
}

class FakeSTM
{
private:
    vector<Goal> goals;
    Point position;
    Goal goal;

    int mode;

    Publisher positionPub;
    Publisher speedPub;
    Publisher robotBlockedPub;
    Publisher finishedPub;

    Subscriber speedSetterSub;
    Subscriber asserManagementSub;
    Subscriber goToAngleSub;
    Subscriber goToSub;
    Subscriber rotateSub;
    Subscriber poseSetterSub;
public:
    FakeSTM();

    void onPing(const std_msgs::Empty::ConstPtr& msg);
    void setSpeed(const can_msgs::Speed::ConstPtr& msg);
    void manage(const can_msgs::STMStatus::ConstPtr& msg);
    
    void goToAngle(const can_msgs::Point::ConstPtr& msg);
    void goTo(const can_msgs::Point::ConstPtr& msg);
    void rotate(const can_msgs::Point::ConstPtr& msg);
    
    void setPose(const can_msgs::Point::ConstPtr& msg);
};

#endif