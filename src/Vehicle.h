//
// Created by uwe_e on 26.01.2020.
//

#ifndef PATH_PLANNING_VEHICLE_H
#define PATH_PLANNING_VEHICLE_H

#include <vector>
#include <iosfwd>

struct Vehicle
{
    double id;
    double x;
    double y;
    double vx;
    double vy;
    double s;
    double d;
};

bool Is_In_Lane(const Vehicle &veh, int lane);

double Vehicle_Speed_Mph(const Vehicle &veh);

std::vector<Vehicle> To_Vehicles(const std::vector<std::vector<double>>& sensor_fusion);

std::ostream& operator<< (std::ostream& os, const Vehicle& veh);

#endif //PATH_PLANNING_VEHICLE_H
