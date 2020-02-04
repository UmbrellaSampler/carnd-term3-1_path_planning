//
// Created by uwe_e on 03.02.2020.
//

#ifndef PATH_PLANNING_PLCL_H
#define PATH_PLANNING_PLCL_H

#include "PLC.h"

class PLCL : public PLC
{
public:
    PLCL(std::shared_ptr<const MapWaypoints> map_way_points, std::shared_ptr<Model> model, int lane,
         std::list<std::shared_ptr<Model>> last_models)
            : PLC(std::move(map_way_points), std::move(model), lane, std::move(last_models))
    {}

    std::vector<std::unique_ptr<State>> Next_States(std::shared_ptr<Model> model) override;

    double Cost() const override;

    std::string Name() const override;

    ~PLCL() = default;

};


#endif //PATH_PLANNING_PLCL_H
