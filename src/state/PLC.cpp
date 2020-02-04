//
// Created by uwe_e on 03.02.2020.
//

#include <config.h>
#include <cmath>
#include "PLC.h"

double PLC::Cost(int target_lane) const
{

    if (target_lane == -1 || target_lane == 3)
    {
        return COST_PLC_LANE_FORBIDDEN;
    }

    double cost = COST_PLC_BASE;
    Vehicle closest;
    if (Closest_Vehicle_In_Lane_Ahead(closest, target_lane))
    {
        double ego_s = model_->ego_state.s;
        if (fabs(closest.s - ego_s) < DIST_SAFE_ADJ_LANE_AHEAD ) {
            cost += COST_LANE_OCCUPIED;
        }
    }

    if (Closest_Vehicle_In_Lane_Behind(closest, target_lane))
    {
        double ego_s = model_->ego_state.s;
        if (fabs(closest.s - ego_s) < DIST_SAFE_ADJ_LANE_BEHIND ) {
            cost += COST_LANE_OCCUPIED;
        }
    }

    return cost;
}
