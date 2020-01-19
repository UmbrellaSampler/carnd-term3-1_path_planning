//
// Created by uwe_e on 19.01.2020.
//

#include "TrajectoryGenerator.h"
#include "spline.h"
#include "helpers.h"
#include <cmath>

TrajectoryGenerator::TrajectoryGenerator(MapWaypoints map_waypoints) :
        map_waypoints_(std::move(map_waypoints))
{}

Trajectory TrajectoryGenerator::NextTrajectory(const Trajectory &previous_traj,
                                               const State &car_state)
{

    // Define the spline
    // Create a list of widely spaced (x, y) waypoints, evenly spaced at 30m
    // Later we will interpolate these waypoints with a spline and fill it in
    // with more points that control speed
    std::vector<double> spline_points_x;
    std::vector<double> spline_points_y;
    const size_t prev_size = previous_traj.path_x.size();

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
        ref_x = previous_traj.path_x[prev_size - 1];
        ref_y = previous_traj.path_y[prev_size - 1];

        double ref_x_prev = previous_traj.path_x[prev_size - 2];
        double ref_y_prev = previous_traj.path_y[prev_size - 2];
        ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

        spline_points_x.push_back(ref_x_prev);
        spline_points_x.push_back(ref_x);

        spline_points_y.push_back(ref_y_prev);
        spline_points_y.push_back(ref_y);
    }

    // In Frenet add evenly 30m spaced points ahead of the starting reference
    std::vector<double> wp0 = getXY(car_state.s + 30, 2 + 4 * lane_);
    std::vector<double> wp1 = getXY(car_state.s + 60, 2 + 4 * lane_);
    std::vector<double> wp2 = getXY(car_state.s + 90, 2 + 4 * lane_);

    spline_points_x.push_back(wp0[0]);
    spline_points_x.push_back(wp1[0]);
    spline_points_x.push_back(wp2[0]);

    spline_points_y.push_back(wp0[1]);
    spline_points_y.push_back(wp1[1]);
    spline_points_y.push_back(wp2[1]);

    for (size_t i = 0; i < spline_points_x.size(); ++i)
    {
        // shift car reference angle to 0 degrees
        double shift_x = spline_points_x[i] - ref_x;
        double shift_y = spline_points_y[i] - ref_y;

        spline_points_x[i] = shift_x * cos(-ref_yaw) - shift_y * sin(-ref_yaw);
        spline_points_y[i] = shift_x * sin(-ref_yaw) + shift_y * cos(-ref_yaw);
    }

    // create a spline
    tk::spline s;
    s.set_points(spline_points_x, spline_points_y);

    // Define the trajectory we will use for the planner
    Trajectory next_traj;

    // Start with all of the previous path points
    next_traj.path_x = previous_traj.path_x;
    next_traj.path_y = previous_traj.path_y;

    // Calculate how to break up spline points so that we travel at our desired reference velocity
    double target_x = 30.0;
    double target_y = s(target_x);
    double target_dist = sqrt(target_x * target_x + target_y * target_y);

    double x_add_on = 0.0;

    // Fill up next trajectory to desired output length
    for (size_t i = 1; i <= 50 - prev_size; ++i)
    {
        double N = target_dist / (0.02 * ref_vel_ / 2.24);
        double x = x_add_on + target_x / N;
        double y = s(x);

        x_add_on = x;

        double x_tmp = x;
        double y_tmp = y;

        // rotate and shift back to global frame
        x = x_tmp * cos(ref_yaw) - y_tmp * sin(ref_yaw);
        y = x_tmp * sin(ref_yaw) + y_tmp * cos(ref_yaw);

        x += ref_x;
        y += ref_y;

        next_traj.path_x.push_back(x);
        next_traj.path_y.push_back(y);
    }


    return next_traj;
}

std::vector<double> TrajectoryGenerator::getXY(double s, double d)
{
    return ::getXY(s, d, map_waypoints_.s, map_waypoints_.x, map_waypoints_.y);
}


