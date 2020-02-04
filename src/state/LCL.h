//
// Created by uwe_e on 03.02.2020.
//

#ifndef PATH_PLANNING_LCL_H
#define PATH_PLANNING_LCL_H

#include "LC.h"

class LCL : public LC
{
public:
    LCL(std::shared_ptr<const MapWaypoints> map_way_points, std::shared_ptr<Model> model, int lane,
        std::list<std::shared_ptr<Model>> last_models)
            : LC(std::move(map_way_points), std::move(model), lane, std::move(last_models))
    {}

    std::vector<std::unique_ptr<State>> Next_States(std::shared_ptr<Model> model) override;
    void Add_Spline_Points(std::vector<double> &spline_points_x, std::vector<double> &spline_points_y) const override;

    std::string Name() const override ;

    ~LCL() = default;

};


#endif //PATH_PLANNING_LCL_H
