#include <ros/ros.h>
#include <ros/package.h>

#include "pathfinder/FindPath.h"

#include "pathfinder/pathfinder_ros_interface.h"
#include "pathfinder/pos_convertor.h"

#include "node_watcher/Node.hpp"
#include "ai_msgs/NodeStatus.h"
#include "ai_msgs/FindPath.h"
#include "ai_msgs/DeclareZone.h"

#include <memory>

using ai_msgs::DeclareZone;
using namespace std;

const double TABLE_WIDTH = 3.0; // Scale corresponding to messages received by the node
const double TABLE_HEIGHT = 2.0; // Scale corresponding to messages received by the node


/**
 * Constructs BarrierSubcribers
 * 
 * @param nodeHandle The node handle used by the node
 * @param topic The topic (or service) name the subscriber has to connect to.
 * @return A unique_ptr containing the subscriber. Will implicitly use std::move.
 */
template<typename T>
unique_ptr<T> constructSubscriber(ros::NodeHandle& nodeHandle, const string& topic);

class PathFinderNode : public Node {
    PathFinderNode() : Node("pathfinder", "ai") {
        // Select the configuration depending on param robot
        ROS_INFO_STREAM("Starting pathfinder...");
        
        // Partialy initialize the convertor
        Pose2D size;
        size.x = TABLE_WIDTH;
        size.y = TABLE_HEIGHT;

        auto convertor = make_shared<PosConvertor>();
        convertor->setInvertedY(true);
        convertor->setRosSize(size);
        
        PathfinderROSInterface pathfinderInterface(convertor);
        
        // Add some obstacle sources
        auto mapSubscriber = this->nh.advertiseService("/ai/pathfinder/declare_zone", &PathFinderNode::declareZone, this);
        /*auto mapSubscriber = constructSubscriber<MapSubscriber>(this->nh, MAP_GET_OBJECTS_SERVER);
        mapSubscriber->setConvertor(convertor);
        pathfinderInterface.addBarrierSubscriber(std::move(mapSubscriber));
        pathfinderInterface.addBarrierSubscriber(constructSubscriber<ObjectsClassifierSubscriber>(nodeHandle, OBJECTS_CLASSIFIER_TOPIC));

        ros::service::waitForService(MAP_GET_OBJECTS_SERVER, 20000);*/

        // Wait for vision job to be done
        this->require("vision", "ai", true);
        this->waitForNodes(200);

        // Configure the main service
        ros::ServiceServer findPathServer = this->nh.advertiseService("/ai/pathfinder/findpath", &PathfinderROSInterface::findPathCallback, &pathfinderInterface);
        
        // Tell other node that we are ready
        this->setNodeStatus(NodeStatus::READY);
    }

    bool declareZone(DeclareZone::Request& req, DeclareZone::Response& res) {
        return false;
    }
};

int main (int argc, char* argv[])
{
    ros::init(argc, argv, "pathfinder_node");
    
//     ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
    ros::NodeHandle nodeHandle;
    
    
    
    ros::spin();
    
    return 0;
}

template<typename T>
unique_ptr<T> constructSubscriber(ros::NodeHandle& nodeHandle, const string& topic)
{
    // Safety margin = 0.15
    unique_ptr<T> subscriber = std::make_unique<T>(0.15);
    subscriber->subscribe(nodeHandle, 100, topic);
    return subscriber;
}
