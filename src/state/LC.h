//
// Created by uwe_e on 03.02.2020.
//

#ifndef PATH_PLANNING_LC_H
#define PATH_PLANNING_LC_H

#include "State.h"

class LC : public State
{
public:
    LC(std::shared_ptr<const MapWaypoints> map_way_points, std::shared_ptr<Model> model, int lane,
       std::list<std::shared_ptr<Model>> last_models)
            : State(std::move(map_way_points), std::move(model), lane, std::move(last_models))
    {}

    std::vector<std::unique_ptr<State>> Next_States(std::shared_ptr<Model> model) = 0;

    double Cost() const override;

    std::string Name() const = 0;

    ~LC() = default;

};

#endif //PATH_PLANNING_LC_H
