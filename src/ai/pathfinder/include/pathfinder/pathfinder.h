#ifndef PATHFINDER_H
#define PATHFINDER_H

#include <ros/console.h>

#include "pathfinder/FindPath.h"

#include "pathfinder/map_storage.h"
#include "pathfinder/pos_convertor.h"

#include "pathfinder/FindPath.h"

#include <vector>
#include <memory>

using pathfinder::FindPath;

/**
 * Main class for the pathfinder algorithm.
 * From arrays reprenting the static and dynamic barriers, it looks for a path between two positions and returns the shortest one if at least one exists.
 */
class Pathfinder
{
public:
    /**
     * Shortcut for path in inside referential and type.
     */
    using Path = std::vector<Pose2D>;
    
    /**
     * Initialize the pathfinder by loading given image file containing static barriers positions, and the size of a rectangle containing all input positions (here the table).
     * @param mapFileName The name of the image file to load.
     * @param dynBarriersMng The dynamic barriers manager already initialized.
     */
    Pathfinder();
    
    /**
     * Try to find a path between the two given positions. The coordinates are directly used in inside referential. It returns false if no paths where found.
     * @param startPos Start position.
     * @param endPos End position.
     * @param path Will contain the shortest path between startPos and endPos if a path was found.
     */
    int findPath(const Pose2D& startPos, const Pose2D& endPos, Path& path);
    
    /**
     * Return the sizes of the internal barrier map.
     */
    Pose2D getMapSize();

private:
    /** Shortcut to define a 2D array of short. **/
    typedef std::vector<std::vector<short> > Vect2DShort;
    /** Shortcut to define a 2D array of bool. vector<bool> is a special type, different from vector<T> **/
    typedef std::vector<std::vector<bool> > Vect2DBool;
    
    /** Manager for loading and saving image files **/
    MapStorage _mapStorage;
    
    /**
     * Add two Pose2D positions into a single new one
     */
    Pose2D add(const Pose2D pos1, const Pose2D pos2) const;

    /**
     * From the end positions tries to reach the start position by increasing step-by-step the distance. For all intermedediate points it stores its distance to count it only one time and to be able after to retrieve the shortest path. Returns true if start position is reached, false else.
     * @param distMap Used to store the distance for all seen position to the end position.
     * @param startPos The start position to reach.
     * @param endPos The end position.
     * @return Tells if a path between start and end postions exists.
     **/
    bool exploreGraph(Vect2DShort& distMap, const Pose2D& startPos, const Pose2D& endPos);
    /**
     * From the start position, find the shortest path to the end position by using the array containing all distances to end position. It returns the complete path from start to end position.
     * @param distMap The 2D array containing distances to end position.
     * @param startPos The start position.
     * @param endPos The end position.
     * @return The complete path between start and end positions.
     */
    Path retrievePath(const Vect2DShort& distMap, const Pose2D& startPos, const Pose2D& endPos);
    /**
     * Removes unnecessary positions in the path by looking for a direct line between two points without meeting any barriers. It returns the cleaned path.
     * @param rawPath The path to clean.
     * @return The cleaned path.
     */
    Path smoothPath(const Path& rawPath);
    
    /** 
     * Check if the given position is in the working referential, and if there is no barriers at the same place.
     **/
    bool isValid(const Pose2D& pos);
    /**
     * Check by "drawing" a line between two positinos if they can be directly connected. Returns true if there is no barriers, false else.
     */
    bool canConnectWithLine(const Pose2D& pA, const Pose2D& pB);
    
    /**
     * Defines all possible directions to move from any positions. May be implemented as constexpression in the future.
     * @return The lists of all allowed movements.
     */
    std::vector<Pose2D> directions() const;
    
    /**
     * Convert the path in the inside type to a string for debugging purposes.
     * @param path The path in inside referential and type.
     * @return The path in string format.
     */
    std::string pathMapToStr(const Path& path);
};

#endif // PATHFINDER_H
