//
//  average_function.cpp
//  2D_DSC
//
//  Created by Asger Nyman Christiansen on 2/14/13.
//  Copyright (c) 2013 Asger Nyman Christiansen. All rights reserved.
//

#include "average_function.h"

void AverageFunc::deform(DeformableSimplicialComplex& dsc)
{
    clock_t init_time = clock();
    for(auto vi = dsc.vertices_begin(); vi != dsc.vertices_end(); ++vi)
    {
        if(dsc.is_movable(*vi))
        {
            dsc.set_destination(*vi, dsc.get_pos(*vi) + VELOCITY * (dsc.avg_pos(*vi, true) - dsc.get_pos(*vi)));
        }
    }
    update_compute_time(clock() - init_time);
    init_time = clock();
    
    dsc.deform();
    
    update_deform_time(clock() - init_time);
}