//
// Created by uwe_e on 02.02.2020.
//

#ifndef PATH_PLANNING_CONFIG_H
#define PATH_PLANNING_CONFIG_H

constexpr int NUM_LANES = 3;

/** Desired velocity of the car */
constexpr double VEL_REF = 46.0;

constexpr double VEL_DIFF_SAFE = 3.0; // 0.224;
constexpr double VEL_DIFF_APPROACH = 2.0; // 0.224;
constexpr double VEL_DIFF_ACC = 3.0; // 0.224;
constexpr double VEL_MAX_ACC = 40.0; // 0.224;

constexpr double DIST_SAFE = 20.0;
constexpr double DIST_APPROACH = DIST_SAFE + 20.0;
constexpr double DIST_SAFE_ADJ_LANE_AHEAD = DIST_APPROACH;
constexpr double DIST_SAFE_ADJ_LANE_BEHIND = 20;

constexpr double COST_KL_BASE = 0.0;
constexpr double COST_KL_VEH_TOO_CLOSE = 10.0;
constexpr double COST_PLC_BASE = 1.0;
constexpr double COST_PLC_LANE_FORBIDDEN = 1000.0;
constexpr double COST_LANE_OCCUPIED = 250.0;

constexpr int MAX_NUM_LAST_STATES = 3;




#endif //PATH_PLANNING_CONFIG_H
