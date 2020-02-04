//
// Created by uwe_e on 03.02.2020.
//

#include "LCR.h"
#include "KeepLane.h"
#include "trajectory_tools.h"

void LCR::Add_Spline_Points(std::vector<double> &spline_points_x, std::vector<double> &spline_points_y) const
{
    // increment lane by 1
    Add_Evenly_Spaced_Points(spline_points_x, spline_points_y, model_, map_waypoints_, lane_ + 1);
}

std::vector<std::unique_ptr<State>> LCR::Next_States(std::shared_ptr<Model> model)
{
    Append_Self_To_Last_Models();
    std::vector<std::unique_ptr<State>> states;
    states.emplace_back(std::make_unique<KeepLane>(map_waypoints_, model, lane_ + 1, last_models_));
    return states;
}

std::string LCR::Name() const
{
    return "LCR";
}