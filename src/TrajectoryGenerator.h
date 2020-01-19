//
// Created by uwe_e on 19.01.2020.
//

#ifndef PATH_PLANNING_TRAJECTORYGENERATOR_H
#define PATH_PLANNING_TRAJECTORYGENERATOR_H

#include <vector>

struct Trajectory
{
    std::vector<double> path_x;
    std::vector<double> path_y;
};

struct State
{
    double x;
    double y;
    double s;
    double d;
    double yaw;
    double speed;
};

struct MapWaypoints {
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> s;
    std::vector<double> dx;
    std::vector<double> dy;
};

class TrajectoryGenerator
{
public:
    explicit TrajectoryGenerator(MapWaypoints map_waypoints);

    /**
     * Computes the next trajectory the car should follow
     *
     * @param previous_traj the remaining trajectory the car has not yet followed during
     *                      last iteration
     * @param car_state the new state of the car
     * @return the computed trajectory the car should follow
     */
    Trajectory NextTrajectory(const Trajectory &previous_traj, const State &car_state);

private:

    std::vector<double> getXY(double s, double d);

    /** Lane of the vehicle */
    int lane_ = 1;

    /** Waypoints of the map */
    MapWaypoints map_waypoints_;

    /** Desired velocity of the car */
    double ref_vel_ = 50.0;
};


#endif //PATH_PLANNING_TRAJECTORYGENERATOR_H
