//
// Created by uwe_e on 03.02.2020.
//

#ifndef PATH_PLANNING_KEEPLANE_H
#define PATH_PLANNING_KEEPLANE_H

#include <utility>

#include "State.h"

class KeepLane final : public State
{
public:
    KeepLane(std::shared_ptr<const MapWaypoints> map_way_points, std::shared_ptr<Model> model, int lane,
             std::list<std::shared_ptr<Model>> last_models)
            : State(std::move(map_way_points), std::move(model), lane, std::move(last_models))
    {}

//    KeepLane(const State& state)
//    : State(state.Map_Way_Points())
//    {}

    std::vector<std::unique_ptr<State>> Next_States(std::shared_ptr<Model> model) override;

    double Cost() const override;

    std::string Name() const override;

    ~KeepLane() = default;

};

#endif //PATH_PLANNING_KEEPLANE_H
