//
// Created by uwe_e on 03.02.2020.
//

#include "PLCL.h"
#include "trajectory_tools.h"
#include "KeepLane.h"
#include "LCL.h"

std::vector<std::unique_ptr<State>> PLCL::Next_States(std::shared_ptr<Model> model)
{
    Append_Self_To_Last_Models();
    std::vector<std::unique_ptr<State>> states;
    states.emplace_back(std::make_unique<KeepLane>(map_waypoints_, model, lane_, last_models_));
    states.emplace_back(std::make_unique<PLCL>(map_waypoints_, model, lane_, last_models_));
    states.emplace_back(std::make_unique<LCL>(map_waypoints_, model, lane_, last_models_));
    return states;
}

double PLCL::Cost() const
{
    return PLC::Cost(lane_ - 1);
}

std::string PLCL::Name() const
{
    return "PLCL";
}
