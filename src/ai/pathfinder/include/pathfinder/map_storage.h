#ifndef MAP_STORAGE_H
#define MAP_STORAGE_H

#include <ros/console.h>
#include <ros/time.h>

#include <vector>

#include "ai_msgs/Shape.h"

class TemporaryShape {
public:
    TemporaryShape(ai_msgs::Shape shape) {
        this->shape = shape;
        this->appearance = ros::Time::now();
    }

    ai_msgs::Shape shape;
    ros::Time appearance;
};

/**
 * Class used to load and save the pathfinder's datas in an image format.
 */
class MapStorage
{
public:
    using Vect2DBool = std::vector<std::vector<bool> >;
    
    MapStorage() = default;
    
    /**
     * Create the allowed positions array.
     * @param width Map width
     * @param height Map height
     * @return A 2D boolean grid with the same size as the loaded image, true is meaning no obstacle in the case.
     */
    Vect2DBool buildAllowedPositions(int width, int height);
    void declareShape(ai_msgs::Shape shape, bool temporary);
    void applyShape(ai_msgs::Shape& shape, Vect2DBool& grid);

    Vect2DBool getAllowedPositions() const;

    int width() const;
    int height() const;
    bool isBlocked(int x, int y) const;
    bool isIn(const ai_msgs::Shape& shape, int x, int y) const;

private:
    Vect2DBool allowedPos;
    std::vector<TemporaryShape> tempShapes;
};


#endif // MAP_STORAGE_H
