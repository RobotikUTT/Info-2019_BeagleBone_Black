#ifndef POS_CONVERTOR_H
#define POS_CONVERTOR_H

#include <geometry_msgs/Pose2D.h>

using geometry_msgs::Pose2D;

/**
 * @brief This class provide functions to convert coordinates between ROS system and the pathfinding one 
 */
class PosConvertor
{
public:
    /**
     * @brief Initialize the convertor. Do nothing yet.
     */
    PosConvertor() = default;
    
    /**
     * @brief Converts a coodinate from ROS system to pathfinding system using the scales.
     * @param rosPos The coodinate in ROS system
     * @return The coordinate in pathfinder system
     */
    Pose2D fromRosToMapPos (Pose2D rosPos) const;
    
    /**
     * @brief Converts a coodinate from pathfinding system to ROS system using the scales.
     * @param mapPos The coodinate in pathfinder system
     * @return The coordinate in ROS system
     */
    Pose2D fromMapToRosPos (Pose2D mapPos) const;
    
    double fromMapToRosDistance (double dist) const;
    
    double fromRosToMapDistance (double dist) const;
    
    // Getters & Setters
    void setSizes (Pose2D sizeRos, Pose2D sizeMap) noexcept { setRosSize(sizeRos); setMapSize(sizeMap); }
    
    void setRosSize(Pose2D sizeRos) noexcept { _sizeRos = sizeRos; }
    void setMapSize(Pose2D sizeMap) noexcept { _sizeMap = sizeMap; }
    
    void setInvertedY(bool invertedY) noexcept { _invertedY = invertedY; }

    double getInternalX(double externalX) const;
    double getInternalY(double externalY) const;
    
private:
    bool _invertedY;
    Pose2D _sizeRos, _sizeMap;
};

#endif // POS_CONVERTOR_H
