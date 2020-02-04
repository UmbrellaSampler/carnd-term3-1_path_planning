//
// Created by uwe_e on 19.01.2020.
//

#ifndef PATH_PLANNING_TRAJECTORYGENERATOR_H
#define PATH_PLANNING_TRAJECTORYGENERATOR_H

#include <vector>
#include <list>
#include "Vehicle.h"
#include "state/State.h"
#include "Types.h"


class TrajectoryGenerator
{
public:
    explicit TrajectoryGenerator(std::shared_ptr<const MapWaypoints> map_waypoints);

    /**
     * Computes the next trajectory the car should follow
     */
    Trajectory Next_Trajectory(std::unique_ptr<Trajectory>&& previous_traj,
                               std::unique_ptr<Model>&& model);

private:

    std::vector<double> getXY(double s, double d);


    /** Lane of the vehicle */
    int lane_ = 1;

    /** Waypoints of the map */
    std::shared_ptr<const MapWaypoints> map_waypoints_;

    /** State of the model */
    std::unique_ptr<State> state_;

};


#endif //PATH_PLANNING_TRAJECTORYGENERATOR_H
