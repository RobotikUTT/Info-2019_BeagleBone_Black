/** @file RobotServices.h
*    @brief Service interface for node readiness for Cpp.
*    
*    
*    @author Alexis CARE
*/
#ifndef ROBOT_SERVICES_H
#define ROBOT_SERVICES_H

#include <string>
#include <ros/ros.h>
#include <ai_msgs/NodeReadiness.h>
/**
 * @defgroup Robot_watcher The Robot_watcher package
 * @{
 */

void service_ready(const std::string, const std::string, const bool, const uint8_t error_code = 0 );

/**
 * @}
 */
#endif
