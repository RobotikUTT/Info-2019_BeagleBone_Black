#include <ros/ros.h>
#include <ros/package.h>
#include <dynamic_reconfigure/server.h>

#include "pathfinder/FindPath.h"

#include "pathfinder/pathfinder_ros_interface.h"
#include "pathfinder/pos_convertor.h"

#include "node_watcher/Node.hpp"
#include "ai_msgs/NodeStatus.h"

#include <memory>

using namespace std;

const string                NAMESPACE_NAME              = "navigation";
const string                NODE_NAME                   = "pathfinder";

const string                FINDPATH_SERVICE_NAME       = NAMESPACE_NAME + "/" + NODE_NAME + "/find_path";
const double                TABLE_WIDTH                 = 3.0; // Scale corresponding to messages received by the node
const double                TABLE_HEIGHT                = 2.0; // Scale corresponding to messages received by the node
const string                PR_MAP_FILE_NAME            = "layer_ground.bmp";
const string                GR_MAP_FILE_NAME            = "layer_pathfinder.bmp"; //"/ros_ws/src/pathfinder/def/map.bmp"; for debug purposes
const string                DEFAULT_ROBOT_NAME          = "gr";

const size_t                SIZE_MAX_QUEUE              = 10;
const double                SAFETY_MARGIN               = 0.15;
const string                MAP_GET_OBJECTS_SERVER      = "static_map/get_container";
const string                OBJECTS_CLASSIFIER_TOPIC    = "recognition/objects_classifier/objects";


/**
 * Constructs BarrierSubcribers
 * 
 * @param nodeHandle The node handle used by the node
 * @param topic The topic (or service) name the subscriber has to connect to.
 * @return A unique_ptr containing the subscriber. Will implicitly use std::move.
 */
template<typename T>
unique_ptr<T> constructSubscriber(ros::NodeHandle& nodeHandle, const string& topic);

/**
 * Retrieve the robot's name from the parameters
 * 
 * @param nodeHandle The node handle used by the node
 * @return The name of the robot
 */
string fetchRobotName(ros::NodeHandle& nodeHandle);


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
        /*auto mapSubscriber = constructSubscriber<MapSubscriber>(this->nh, MAP_GET_OBJECTS_SERVER);
        mapSubscriber->setConvertor(convertor);
        pathfinderInterface.addBarrierSubscriber(std::move(mapSubscriber));
        pathfinderInterface.addBarrierSubscriber(constructSubscriber<ObjectsClassifierSubscriber>(nodeHandle, OBJECTS_CLASSIFIER_TOPIC));

        ros::service::waitForService(MAP_GET_OBJECTS_SERVER, 20000);*/

        // Wait for vision job to be done
        this->require("vision", "ai", true);
        this->waitForNodes(200);

        // Configure the main service
        ros::ServiceServer findPathServer = this->nh.advertiseService(FINDPATH_SERVICE_NAME, &PathfinderROSInterface::findPathCallback, &pathfinderInterface);
        
        // Tell other node that we are ready
        this->setNodeStatus(NodeStatus::READY);
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
    unique_ptr<T> subscriber = std::make_unique<T>(SAFETY_MARGIN);
    subscriber->subscribe(nodeHandle, SIZE_MAX_QUEUE, topic);
    return subscriber;
}
