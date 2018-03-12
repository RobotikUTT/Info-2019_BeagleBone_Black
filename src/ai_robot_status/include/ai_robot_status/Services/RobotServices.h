#ifndef ROBOT_SERVICES_H
#define ROBOT_SERVICES_H

#include <string>
#include <ros/ros.h>
#include <ai_robot_status/NodeReadiness.h>

void service_ready(const std::string, const std::string, const bool );
#endif