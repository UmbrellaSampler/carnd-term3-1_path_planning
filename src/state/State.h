//
// Created by uwe_e on 26.01.2020.
//

#ifndef PATH_PLANNING_STATE_H
#define PATH_PLANNING_STATE_H

#include <utility>
#include <vector>
#include <list>
#include <string>
#include <memory>
#include "spline.h"
#include "Types.h"

//enum class State {
//    Ready,
//    LK,
//    PLCL,
//    PLCR,
//    LCL,
//    LCR
//};

class State
{
public:
    State(std::shared_ptr<const MapWaypoints> map_way_points, std::shared_ptr<Model> model, int lane,
          std::list<std::shared_ptr<Model>> last_models)
            : map_waypoints_(std::move(map_way_points)),
              model_(std::move(model)), lane_(lane), last_models_(std::move(last_models))
    {}

    State (const State& state) = default;

    virtual ~State() = default;

    virtual std::vector<std::unique_ptr<State>> Next_States(std::shared_ptr<Model> model_) = 0;

    virtual double Cost() const = 0;

    virtual void Add_Spline_Points(std::vector<double> &spline_points_x, std::vector<double> &spline_points_y) const;

    // For debugging
    virtual std::string Name() const = 0;

    Trajectory Next_Trajectory(const std::unique_ptr<Trajectory> &prev_traj) const;

protected:

    void Append_Self_To_Last_Models();

    bool Vehicle_Ahead_Too_Close(double dist) const;
    bool Too_Close(const Vehicle &vehicle, double dist) const;

    std::vector<int> Vehicles_In_Lane(int lane) const;
    std::vector<int> Vehicles_In_Lane_Ahead(int lane) const;
    bool Closest_Vehicle_In_Lane_Ahead(Vehicle &closest, int lane) const;
    std::vector<int> Vehicles_In_Lane_Behind(int lane) const;
    bool Closest_Vehicle_In_Lane_Behind(Vehicle &closest, int lane) const;


    Trajectory Break_Up_Spline_Points(std::vector<double> &spline_points_x,
                                      std::vector<double> &spline_points_y,
                                      const std::unique_ptr<Trajectory> &previous_traj,
                                      double ref_x,
                                      double ref_y,
                                      double ref_yaw,
                                      double ego_vel) const;

    std::shared_ptr<const MapWaypoints> map_waypoints_;

    std::shared_ptr<Model> model_;

    int lane_;

    std::list<std::shared_ptr<Model>> last_models_;
};

#endif //PATH_PLANNING_STATE_H
