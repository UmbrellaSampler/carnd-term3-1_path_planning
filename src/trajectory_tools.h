//
// Created by uwe_e on 02.02.2020.
//

#ifndef PATH_PLANNING_TRAJECTORY_TOOLS_H
#define PATH_PLANNING_TRAJECTORY_TOOLS_H

#include "Types.h"
#include <vector>
#include <memory>

double Add_Ref_Points(
        std::vector<double> &spline_points_x,
        std::vector<double> &spline_points_y,
        const std::unique_ptr<Trajectory>& previous_traj,
        const std::shared_ptr<Model>& model);

void Add_Evenly_Spaced_Points(
        std::vector<double> &spline_points_x,
        std::vector<double> &spline_points_y,
        const std::shared_ptr<Model> &model,
        const std::shared_ptr<const MapWaypoints> &map_waypoints,
        int lane);

void Rotate_Points(
        std::vector<double> &spline_points_x,
        std::vector<double> &spline_points_y,
        double ref_yaw);

#endif //PATH_PLANNING_TRAJECTORY_TOOLS_H
