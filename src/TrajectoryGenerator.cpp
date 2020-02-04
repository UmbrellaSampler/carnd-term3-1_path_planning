//
// Created by uwe_e on 19.01.2020.
//

#include "TrajectoryGenerator.h"
#include "helpers.h"
#include "KeepLane.h"
#include <thread>

TrajectoryGenerator::TrajectoryGenerator(std::shared_ptr<const MapWaypoints> map_waypoints) :
        map_waypoints_(map_waypoints)
{
    state_ = std::move(std::make_unique<KeepLane>(map_waypoints, nullptr, 1,
                                                  std::list<std::shared_ptr<Model>>()));
}

Trajectory TrajectoryGenerator::Next_Trajectory(std::unique_ptr<Trajectory> &&previous_traj,
                                                std::unique_ptr<Model> &&model)
{
    // Define the spline
    // Create a list of widely spaced (x, y) waypoints, evenly spaced at 30m
    // Later we will interpolate these waypoints with a spline and fill it in
    // with more points that control speed

    std::shared_ptr<Model> sh_model = std::move(model);

    // Get the state with minimal cost
    std::vector<std::pair<std::unique_ptr<State>, double>> states_and_costs;
    for (auto &state :  state_->Next_States(sh_model))
    {
        double cost = state->Cost();
        states_and_costs.emplace_back(std::move(state), cost);
    }

    auto min_el = std::min_element(states_and_costs.begin(), states_and_costs.end(),
                     [](const std::pair<std::unique_ptr<State>, double>& s0,
                             const std::pair<std::unique_ptr<State>, double>& s1) {
                         return s0.second < s1.second;
                     });
    state_ = std::move(min_el->first);
    return state_->Next_Trajectory(previous_traj);

}


