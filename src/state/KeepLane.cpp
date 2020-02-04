//
// Created by uwe_e on 03.02.2020.
//

#include "KeepLane.h"
#include "trajectory_tools.h"
#include "config.h"
#include "PLCL.h"
#include "PLCR.h"

std::vector<std::unique_ptr<State>> KeepLane::Next_States(std::shared_ptr<Model> model)
{
    Append_Self_To_Last_Models();
    std::vector<std::unique_ptr<State>> states;
    states.emplace_back(std::make_unique<KeepLane>(map_waypoints_, model, lane_, last_models_));
    states.emplace_back(std::make_unique<PLCL>(map_waypoints_, model, lane_, last_models_));
    states.emplace_back(std::make_unique<PLCR>(map_waypoints_, model, lane_, last_models_));
    return states;
}

double KeepLane::Cost() const
{
    double cost = COST_KL_BASE;
    if (Vehicle_Ahead_Too_Close(DIST_APPROACH))
    {
        cost += COST_KL_VEH_TOO_CLOSE;
    }
    return cost;
}

std::string KeepLane::Name() const
{
    return "KL";
}
