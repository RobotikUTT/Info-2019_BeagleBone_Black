#include "pathfinder/pathfinder_ros_interface.h"

using namespace std;

PathfinderROSInterface::PathfinderROSInterface(std::shared_ptr<PosConvertor> convertor)
{
    convertor_ = convertor;
    
    auto mapSize = pathfinderPtr_->getMapSize();
    if (mapSize.x == 0)
        ROS_FATAL("Allowed positions empty. Cannot define a scale. Please restart the node, it may crash soon.");
    else
    {
        if (mapSize.x * mapSize.y > 200000) {
            ROS_WARN("Map image is big, the pathfinder may be very slow ! (150x100px works fine)");
        }

        convertor_->setMapSize(mapSize);
    }
}


bool PathfinderROSInterface::findPathCallback(pathfinder::FindPath::Request& req, pathfinder::FindPath::Response& rep)
{
    Pathfinder::Path path;
    
    ROS_INFO_STREAM("Received request from " << req.posStart << " to " << req.posEnd);
    
    auto startPos = convertor_->fromRosToMapPos(req.posStart);
    auto endPos = convertor_->fromRosToMapPos(req.posEnd);
    
    rep.return_code = pathfinderPtr_->findPath(startPos, endPos, path);

    // In case path is found
    if (rep.return_code == FindPath::Response::PATH_FOUND) {
        for (const Pose2D& pos : path)
            rep.path.push_back(convertor_->fromMapToRosPos(pos));
        rep.return_code = rep.PATH_FOUND;
        rep.path.front() = req.posStart;
        rep.path.back() = req.posEnd;
        ROS_DEBUG_STREAM("Answering: " << pathRosToStr_(rep.path));
    }
    
    return true;
}

string PathfinderROSInterface::pathRosToStr_(const vector<geometry_msgs::Pose2D>& path)
{
    ostringstream os;
    string str = "[";
    for (const geometry_msgs::Pose2D& pos : path)
        os << pos << ", ";
    str += os.str();
    if (str.length() > 2)
        str.erase(str.end()-2, str.end());
    str += "]";
    return str;
}
