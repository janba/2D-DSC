//
//  Deformabel Simplicial Complex (DSC) method
//  Copyright (C) 2013  Technical University of Denmark
//
//  This program is free software: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  See licence.txt for a copy of the GNU General Public License.

#include "velocity_function.h"

namespace DSC2D
{
    
    bool VelocityFunc::is_motion_finished(DeformableSimplicialComplex& dsc)
    {
        if (time_step == MAX_TIME_STEPS) {
            return true;
        }
        std::vector<CGLA::Vec2d> pos = dsc.get_design_variable_positions();
        for (auto &p : pos)
        {
            bool match = false;
            for (int i = 0; i + 1 < pos_old.size(); i += 2)
            {
                if (Util::min_dist_sqr(pos_old[i], pos_old[i+1], p) < ACCURACY*ACCURACY)
                {
                    match = true;
                    break;
                }
            }
            if (!match) {
                std::cout << "Stopping criteria: Position " << p << " has moved." << std::endl;
                pos_old = dsc.get_interface_edge_positions();
                return false;
            }
        }
        pos_old = dsc.get_interface_edge_positions();
        return true;
    }
    
}