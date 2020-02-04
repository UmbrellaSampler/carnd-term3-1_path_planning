//
// Created by uwe_e on 02.02.2020.
//

#include "trajectory_tools.h"
#include "helpers.h"
#include "spline.h"
#include "config.h"

#include <cmath>


double Add_Ref_Points(
        std::vector<double> &spline_points_x,
        std::vector<double> &spline_points_y,
        const std::unique_ptr<Trajectory> &previous_traj,
        const std::shared_ptr<Model> &model)
{
    const size_t prev_size = previous_traj->path_x.size();

    const EgoState &car_state = model->ego_state;
    double ref_x = car_state.x;
    double ref_y = car_state.y;
    double ref_yaw = deg2rad(car_state.yaw);

    // If previous size is almost empty, use the car as starting reference
    if (prev_size < 2)
    {
        // Use two points that make the path tangent to the car
        double ref_x_prev = ref_x - cos(ref_yaw);
        double ref_y_prev = ref_y - sin(ref_yaw);

        spline_points_x.push_back(ref_x_prev);
        spline_points_x.push_back(ref_x);

        spline_points_y.push_back(ref_y_prev);
        spline_points_y.push_back(ref_y);
    }
    else // Use the previous path's end point as starting reference
    {
        // Redefine reference state as previous path end point
        ref_x = previous_traj->path_x[prev_size - 1];
        ref_y = previous_traj->path_y[prev_size - 1];

        double ref_x_prev = previous_traj->path_x[prev_size - 2];
        double ref_y_prev = previous_traj->path_y[prev_size - 2];
        ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

        spline_points_x.push_back(ref_x_prev);
        spline_points_x.push_back(ref_x);

        spline_points_y.push_back(ref_y_prev);
        spline_points_y.push_back(ref_y);
    }

    return ref_yaw;
}

void Add_Evenly_Spaced_Points(
        std::vector<double> &spline_points_x,
        std::vector<double> &spline_points_y,
        const std::shared_ptr<Model> &model,
        const std::shared_ptr<const MapWaypoints> &map_waypoints,
        int lane)
{
    const EgoState &car_state = model->ego_state;
    std::vector<double> wp0 = getXY(car_state.s + 60, 2 + 4 * lane, map_waypoints->s, map_waypoints->x,
                                    map_waypoints->y);
    std::vector<double> wp1 = getXY(car_state.s + 90, 2 + 4 * lane, map_waypoints->s, map_waypoints->x,
                                    map_waypoints->y);
    std::vector<double> wp2 = getXY(car_state.s + 120, 2 + 4 * lane, map_waypoints->s, map_waypoints->x,
                                    map_waypoints->y);

    spline_points_x.push_back(wp0[0]);
    spline_points_x.push_back(wp1[0]);
    spline_points_x.push_back(wp2[0]);

    spline_points_y.push_back(wp0[1]);
    spline_points_y.push_back(wp1[1]);
    spline_points_y.push_back(wp2[1]);
}

void Rotate_Points(
        std::vector<double> &spline_points_x,
        std::vector<double> &spline_points_y,
        double ref_yaw)
{
    double ref_x = spline_points_x[1];
    double ref_y = spline_points_y[1];
    for (size_t i = 0; i < spline_points_x.size(); ++i)
    {
        // shift car reference angle to 0 degrees
        double shift_x = spline_points_x[i] - ref_x;
        double shift_y = spline_points_y[i] - ref_y;

        spline_points_x[i] = shift_x * cos(-ref_yaw) - shift_y * sin(-ref_yaw);
        spline_points_y[i] = shift_x * sin(-ref_yaw) + shift_y * cos(-ref_yaw);
    }
}