//
// Created by uwe_e on 01.02.2020.
//

#ifndef PATH_PLANNING_TYPES_H
#define PATH_PLANNING_TYPES_H

#include <vector>
#include "Vehicle.h"

struct Trajectory
{
    std::vector<double> path_x;
    std::vector<double> path_y;
};

struct EgoState
{
    double x;
    double y;
    double s;
    double d;
    double yaw;
    double speed;
};

struct MapWaypoints
{
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> s;
    std::vector<double> dx;
    std::vector<double> dy;
};

struct Sensors
{
    std::vector<Vehicle> vehicles;
};

struct Model {
    EgoState ego_state;
    Sensors sensors;
};

#endif //PATH_PLANNING_TYPES_H
