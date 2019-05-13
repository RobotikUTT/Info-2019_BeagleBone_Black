#include "pathfinder/map_storage.h"

using namespace std;

MapStorage::Vect2DBool MapStorage::buildAllowedPositions(int width, int height)
{
    Vect2DBool allowedPos;
    for (unsigned int line = 0; line < height; line++)
    {
        allowedPos.emplace_back();
        for (unsigned int column = 0; column < width; column++)
            allowedPos[line].push_back(true);
    }
    
    ROS_DEBUG_STREAM("MapStorage: Done, map size is " << allowedPos.front().size() << "x" << allowedPos.size());
    return allowedPos;
}