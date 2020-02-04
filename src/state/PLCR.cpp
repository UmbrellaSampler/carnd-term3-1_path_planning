//
// Created by uwe_e on 03.02.2020.
//

#include "PLCR.h"
#include "trajectory_tools.h"
#include "KeepLane.h"
#include "LCR.h"

std::vector<std::unique_ptr<State>> PLCR::Next_States(std::shared_ptr<Model> model)
{
    Append_Self_To_Last_Models();
    std::vector<std::unique_ptr<State>> states;
    states.emplace_back(std::make_unique<KeepLane>(map_waypoints_, model, lane_, last_models_));
    states.emplace_back(std::make_unique<PLCR>(map_waypoints_, model, lane_, last_models_));
    states.emplace_back(std::make_unique<LCR>(map_waypoints_, model, lane_, last_models_));
    return states;
}

double PLCR::Cost() const
{
    return PLC::Cost(lane_ + 1);
}

std::string PLCR::Name() const
{
    return "PLCR";
}