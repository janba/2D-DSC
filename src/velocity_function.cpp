//
//  velocity_function.cpp
//  2D_DSC
//
//  Created by Asger Nyman Christiansen on 10/1/12.
//  Copyright (c) 2012 DTU Informatics. All rights reserved.
//

#include "velocity_function.h"

bool VelocityFunc::is_motion_finished(DeformableSimplicialComplex& dsc)
{
    if (time_step == MAX_TIME_STEPS) {
        return true;
    }
    std::vector<CGLA::Vec2d> pos = dsc.get_design_variable_positions();
    for (auto p = pos.begin(); p != pos.end(); p++)
    {
        bool match = false;
        for (int i = 0; i+1 < pos_old.size(); i += 2)
        {
            if (Util::min_dist(pos_old[i], pos_old[i+1], *p) < ACCURACY)
            {
                match = true;
                break;
            }
        }
        if (!match) {
            std::cout << "Stopping criteria: Position " << *p << " has moved." << std::endl;
            pos_old = dsc.get_interface_edge_positions();
            return false;
        }
    }
    pos_old = dsc.get_interface_edge_positions();
    return true;
}
