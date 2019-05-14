#include <ros/ros.h>
#include <ros/package.h>

#include "pathfinder/FindPath.h"

#include "pathfinder/pathfinder.h"
#include "pathfinder/pos_convertor.h"

#include "node_watcher/Node.hpp"
#include "ai_msgs/NodeStatus.h"
#include "ai_msgs/DeclareZone.h"

#include <memory>

using ai_msgs::DeclareZone;
using namespace std;

const double TABLE_WIDTH = 3.0; // Scale corresponding to messages received by the node
const double TABLE_HEIGHT = 2.0; // Scale corresponding to messages received by the node

class PathFinderNode : public Node {

private:
    std::shared_ptr<PosConvertor> convertor;
    /**
     * Pointer to the main algorithm
     */
    std::unique_ptr<Pathfinder> pathfinderPtr_;
    ros::ServiceServer findPathServer;

public:
    PathFinderNode() : Node("pathfinder", "ai") {
        // Partialy initialize the convertor
        Pose2D size;
        size.x = TABLE_WIDTH;
        size.y = TABLE_HEIGHT;

        this->convertor = make_shared<PosConvertor>();
        this->convertor->setInvertedY(true);
        this->convertor->setRosSize(size);

        // Init pathfinder
        this->pathfinderPtr_ = make_unique<Pathfinder>();
        
        // Assert correct size
        auto mapSize = pathfinderPtr_->getMapSize();
        if (mapSize.x == 0) {
            ROS_FATAL("Allowed positions empty. Cannot define a scale. Please restart the node, it may crash soon.");
        } else {
            if (mapSize.x * mapSize.y > 200000) {
                ROS_WARN("Map image is big, the pathfinder may be very slow ! (150x100px works fine)");
            }

            this->convertor->setMapSize(mapSize);
        }
        
        // Obstacle declaration service
        auto mapSubscriber = this->nh.advertiseService("/ai/pathfinder/declare_zone", &PathFinderNode::declareZone, this);

        // Wait for vision job to be done
        this->require("map_handler", "ai", true);
        this->waitForNodes(10, false);
    }

    void onWaitingResult(bool success) override {
        if (!success) {
            ROS_WARN_STREAM("Unable to wait for 'map_handler' to be ready, some data on the map might be missing...");
        }

        // Then start pathfinding service
        this->findPathServer = this->nh.advertiseService("/ai/pathfinder/findpath", &PathFinderNode::findPathCallback, this);

        // Tell other node that we are ready
        this->setNodeStatus(NodeStatus::READY);
    }


    bool findPathCallback(pathfinder::FindPath::Request& req, pathfinder::FindPath::Response& rep) {
        Pathfinder::Path path;
        
        ROS_INFO_STREAM("Received request from " << req.posStart << " to " << req.posEnd);
        
        auto startPos = this->convertor->fromRosToMapPos(req.posStart);
        auto endPos = this->convertor->fromRosToMapPos(req.posEnd);
        
        rep.return_code = pathfinderPtr_->findPath(startPos, endPos, path);

        // In case path is found
        if (rep.return_code == pathfinder::FindPath::Response::PATH_FOUND) {
            for (const Pose2D& pos : path) {
                rep.path.push_back(this->convertor->fromMapToRosPos(pos));
            }

            rep.return_code = rep.PATH_FOUND;
            rep.path.front() = req.posStart;
            rep.path.back() = req.posEnd;

            ROS_DEBUG_STREAM("Answering: " << pathRosToStr_(rep.path));
        }
        
        return true;
    }

    string pathRosToStr_(const vector<geometry_msgs::Pose2D>& path) {
        ostringstream os;
        string str = "[";
        for (const geometry_msgs::Pose2D& pos : path) {
            os << pos << ", ";
        }
        str += os.str();
        
        if (str.length() > 2) {
            str.erase(str.end()-2, str.end());
        }
        str += "]";

        return str;
    }

    bool declareZone(DeclareZone::Request& req, DeclareZone::Response& res) {
        // Pass declaration to map
        this->pathfinderPtr_->getMapStorage().declareShape(req.zone, req.temporary);

        // Mark as done
        return true;
    }
    
};

int main (int argc, char* argv[])
{
    ros::init(argc, argv, "pathfinder_node");
    
//     ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
    PathFinderNode node;
    ros::spin();
    
    return 0;
}