#ifndef MAP_STORAGE_H
#define MAP_STORAGE_H

#include <ros/console.h>

#include <vector>


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

};


#endif // MAP_STORAGE_H
