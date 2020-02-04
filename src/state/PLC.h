//
// Created by uwe_e on 03.02.2020.
//

#ifndef PATH_PLANNING_PLC_H
#define PATH_PLANNING_PLC_H

#include "State.h"

class PLC : public State
{
public:
    PLC(std::shared_ptr<const MapWaypoints> map_way_points, std::shared_ptr<Model> model, int lane,
        std::list<std::shared_ptr<Model>> last_models)
            : State(std::move(map_way_points), std::move(model), lane, std::move(last_models))
    {}

    std::vector<std::unique_ptr<State>> Next_States(std::shared_ptr<Model> model) = 0;

    double Cost() const = 0;

    std::string Name() const = 0;

    ~PLC() = default;

protected:
    double Cost(int target_lane) const;
};


#endif //PATH_PLANNING_PLC_H
