//
// Created by uwe_e on 26.01.2020.
//

#include <cmath>
#include <algorithm>
#include <iostream>
#include "config.h"
#include "State.h"
#include "trajectory_tools.h"

using std::vector;

void State::Add_Spline_Points(std::vector<double> &spline_points_x, std::vector<double> &spline_points_y) const
{
    Add_Evenly_Spaced_Points(spline_points_x, spline_points_y, model_, map_waypoints_, lane_);
}

bool State::Vehicle_Ahead_Too_Close(double dist) const
{
    bool too_close = false;

    Vehicle closest;
    if (Closest_Vehicle_In_Lane_Ahead(closest, lane_))
    {
        too_close = closest.s - model_->ego_state.s < dist;
    }
    return too_close;
}

bool State::Too_Close(const Vehicle &vehicle, double dist) const
{
    return (vehicle.s - model_->ego_state.s) <= dist;
}

std::vector<int> State::Vehicles_In_Lane(int lane) const
{
    std::vector<int> vehicle_idx;

    int i = 0;
    for (auto const &v : model_->sensors.vehicles)
    {
        if (Is_In_Lane(v, lane))
        {
            vehicle_idx.push_back(i);
        }
        ++i;
    }
    return vehicle_idx;
}

std::vector<int> State::Vehicles_In_Lane_Ahead(int lane) const
{
    std::vector<int> vehicle_in_lane = Vehicles_In_Lane(lane);
    std::vector<int> vehicle_in_lane_ahead;
    std::copy_if(
            vehicle_in_lane.begin(), vehicle_in_lane.end(), std::back_inserter(vehicle_in_lane_ahead), [this](int i) {
                auto const &vehicles = model_->sensors.vehicles;
                double ego_s = model_->ego_state.s;
                return vehicles[i].s > ego_s;
            }
    );
    return vehicle_in_lane_ahead;
}

bool State::Closest_Vehicle_In_Lane_Ahead(Vehicle &closest, int lane) const
{
    std::vector<int> vehicle_in_lane_ahead = Vehicles_In_Lane_Ahead(lane);

    if (vehicle_in_lane_ahead.empty())
    {
        return false;
    }
    else
    {
        auto min_dist_veh = std::min_element(vehicle_in_lane_ahead.begin(),
                                             vehicle_in_lane_ahead.end(),
                                             [this](int a, int b) {
                                                 auto const &vehicles = model_->sensors.vehicles;
                                                 return vehicles[a].s < vehicles[b].s;
                                             });
        closest = model_->sensors.vehicles[*min_dist_veh];
        return true;
    }
}

std::vector<int> State::Vehicles_In_Lane_Behind(int lane) const
{
    std::vector<int> vehicle_in_lane = Vehicles_In_Lane(lane);
    std::vector<int> vehicle_in_lane_ahead;
    std::copy_if(
            vehicle_in_lane.begin(), vehicle_in_lane.end(), std::back_inserter(vehicle_in_lane_ahead), [this](int i) {
                auto const &vehicles = model_->sensors.vehicles;
                double ego_s = model_->ego_state.s;
                return vehicles[i].s < ego_s;
            }
    );
    return vehicle_in_lane_ahead;
}

bool State::Closest_Vehicle_In_Lane_Behind(Vehicle &closest, int lane) const
{
    std::vector<int> vehicle_in_lane_ahead = Vehicles_In_Lane_Behind(lane);

    if (vehicle_in_lane_ahead.empty())
    {
        return false;
    }
    else
    {
        auto min_dist_veh = std::max_element(vehicle_in_lane_ahead.begin(),
                                             vehicle_in_lane_ahead.end(),
                                             [this](int a, int b) {
                                                 auto const &vehicles = model_->sensors.vehicles;
                                                 return vehicles[a].s < vehicles[b].s;
                                             });
        closest = model_->sensors.vehicles[*min_dist_veh];
        return true;
    }
}

Trajectory State::Break_Up_Spline_Points(std::vector<double> &spline_points_x, std::vector<double> &spline_points_y,
                                         const std::unique_ptr<Trajectory> &previous_traj, double ref_x, double ref_y,
                                         double ref_yaw, double ego_vel) const
{
    // Velocity control
    double max_vel_diff = VEL_DIFF_SAFE;
    // acceleration should no exceed limit
    if (last_models_.size() >= 2)
    {
        double ego_vel_2 = (++(last_models_.begin()))->get()->ego_state.speed / 2.23694; // mph -> m/s
        double ego_vel_0 = ego_vel / 2.23694;
        double acc_last = fabs(ego_vel_0 - ego_vel_2) / (2 * 0.02);

        max_vel_diff = std::min(VEL_MAX_ACC * 0.02, fabs(VEL_MAX_ACC - acc_last) * 0.02);
        max_vel_diff *= 2.23694; // m/s -> mph
    }

    double target_vel_mph = ego_vel;
    Vehicle closest;
    if (Closest_Vehicle_In_Lane_Ahead(closest, lane_))
    {
        double veh_vel = Vehicle_Speed_Mph(closest);
        // Adapt speed
        if (Too_Close(closest, DIST_SAFE))
        {
//            target_vel_mph -= VEL_DIFF_SAFE;
            target_vel_mph -= max_vel_diff;
        }
        else if (Too_Close(closest, DIST_APPROACH))
        {
            if (fabs(veh_vel - ego_vel) < 2.0)
            {
                target_vel_mph = veh_vel;
            }
            else if (veh_vel < ego_vel)
            {
//                target_vel_mph -= VEL_DIFF_APPROACH;
                target_vel_mph -= max_vel_diff;
            }
            else
            {
//                target_vel_mph += VEL_DIFF_APPROACH;
                target_vel_mph += max_vel_diff;
            }
        }
        else if (ego_vel < VEL_REF)
        {
//            target_vel_mph += VEL_DIFF_ACC;
            target_vel_mph += max_vel_diff;
        }
    }
    else if (ego_vel < VEL_REF)
    {
//        target_vel_mph += VEL_DIFF_ACC;
        target_vel_mph += max_vel_diff;
    }

    tk::spline spline;
    spline.set_points(spline_points_x, spline_points_y);

    // Start with all of the previous path points
    Trajectory next_traj;
    next_traj.path_x = previous_traj->path_x;
    next_traj.path_y = previous_traj->path_y;

    // Calculate how to break up spline points so that we travel at our desired reference velocity
    double target_x = 30.0;
    double target_y = spline(target_x);
    double target_dist = sqrt(target_x * target_x + target_y * target_y);

    double x_add_on = 0.0;

    // Fill up next trajectory to desired output length
    const size_t prev_size = previous_traj->path_x.size();
    for (size_t i = 1; i <= 50 - prev_size; ++i)
    {
        double N = target_dist / (0.02 * target_vel_mph / 2.24);
        double x = x_add_on + target_x / N;
        double y = spline(x);

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


Trajectory State::Next_Trajectory(const std::unique_ptr<Trajectory> &prev_traj) const
{
    std::vector<double> spline_points_x;
    std::vector<double> spline_points_y;

    double ref_yaw = Add_Ref_Points(spline_points_x, spline_points_y, prev_traj, model_);
    double ref_x = spline_points_x[1];
    double ref_y = spline_points_y[1];
    double ref_vel = model_->ego_state.speed;

    this->Add_Spline_Points(spline_points_x, spline_points_y);
    Rotate_Points(spline_points_x, spline_points_y, ref_yaw);
    auto trajectory = Break_Up_Spline_Points(spline_points_x, spline_points_y, prev_traj,
                                             ref_x, ref_y, ref_yaw, ref_vel);
    return trajectory;
}

void State::Append_Self_To_Last_Models()
{
    if (this->model_ != nullptr)
    {
        last_models_.push_front(this->model_);
        if (last_models_.size() > MAX_NUM_LAST_STATES)
        {
            last_models_.pop_back();
        }
    }
}

//vector<State> Get_Next_States(State state, int lane, int lanes_available)
//{
//    vector<State> states;
//    states.push_back(State::LK);
//    if(state == State::LK) {
//        states.push_back(State::PLCL);
//        states.push_back(State::PLCR);
//    } else if (state == State::PLCL) {
//        if (lane != lanes_available - 1) {
//            states.push_back(State::PLCL);
//            states.push_back(State::LCL);
//        }
//    } else if (state == State::PLCR) {
//        if (lane != 0) {
//            states.push_back(State::PLCR);
//            states.push_back(State::LCR);
//        }
//    }
//
//    // If state is "LCL" or "LCR", then just return "KL"
//    return states;
//}
