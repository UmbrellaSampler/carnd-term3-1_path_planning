//
// Created by uwe_e on 26.01.2020.
//

#include "Vehicle.h"
#include <ostream>
#include <iomanip>
#include <cmath>

using ::std::vector;

bool Is_In_Lane(const Vehicle &veh, int lane)
{
    return veh.d > (2 + 4 * lane - 2) && veh.d < (2 + 4 * lane + 2);
}

double Vehicle_Speed_Mph(const Vehicle &veh)
{
    return sqrt(veh.vx * veh.vx + veh.vy * veh.vy) * 2.23694;
}

vector<Vehicle> To_Vehicles(const vector<vector<double>> &sensor_fusion)
{
    vector<Vehicle> vehicles;
    for (size_t i = 0; i < sensor_fusion.size(); ++i)
    {
        vehicles.push_back({sensor_fusion[i][0],
                            sensor_fusion[i][1],
                            sensor_fusion[i][2],
                            sensor_fusion[i][3],
                            sensor_fusion[i][4],
                            sensor_fusion[i][5],
                            sensor_fusion[i][6]});
    }
    return vehicles;
}

std::ostream &operator<<(std::ostream &os, const Vehicle &veh)
{
    using std::setprecision;

    constexpr int prec = 3;
    os << setprecision(prec) << "id=" << veh.id << "; ";
    os << setprecision(prec) << "x=" << veh.x << "; ";
    os << setprecision(prec) << "y=" << veh.y << "; ";
    os << setprecision(prec) << "vx=" << veh.vx << "; ";
    os << setprecision(prec) << "vy=" << veh.vy << "; ";
    os << setprecision(prec) << "s=" << veh.s << "; ";
    os << setprecision(prec) << "d=" << veh.d << ";";
    return os;
}
