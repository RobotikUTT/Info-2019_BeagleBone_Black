#include "pathfinder/pos_convertor.h"

using namespace std;

Pose2D PosConvertor::fromRosToMapPos (Pose2D rosPos) const
{
    Pose2D res;
    res.x = rosPos.x * (_sizeMap.x / _sizeRos.x);
    res.y = rosPos.y * (_sizeMap.y / _sizeRos.y);
    
    if (_invertedY)
        res.y = _sizeMap.y - res.y;
    

    return res;
}


Pose2D PosConvertor::fromMapToRosPos (Pose2D mapPos) const
{
    Pose2D res;
    res.x = mapPos.x * (_sizeRos.x / _sizeMap.x);
    res.y = mapPos.y * (_sizeRos.y / _sizeMap.y);
    
    if (_invertedY)
        res.y = _sizeRos.y - res.y;
    
    return res;
}

double PosConvertor::getInternalX(double externalX) const {
    return externalX * _sizeMap.x / _sizeRos.x;
}

double PosConvertor::getInternalY(double externalY) const {
    return externalY * _sizeMap.y / _sizeRos.y;
}

double PosConvertor::fromMapToRosDistance(double dist) const
{
    double xCoef = _sizeMap.x/_sizeRos.x;
    double yCoef = _sizeMap.y/_sizeRos.y;
    // We assume that the scale on x and y is the same, we take the linear average to have a better precision.
    double coef = (xCoef + yCoef)/2;
    return dist/coef;
}

double PosConvertor::fromRosToMapDistance(double dist) const
{
    double xCoef = _sizeMap.x/_sizeRos.x;
    double yCoef = _sizeMap.y/_sizeRos.y;
    // We assume that the scale on x and y is the same, we take the linear average to have a better precision.
    double coef = (xCoef + yCoef)/2;
    return dist*coef;
}

