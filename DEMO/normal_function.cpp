//
//  normal_function.cpp
//  2D_DSC
//
//  Created by Asger Nyman Christiansen on 2/14/13.
//  Copyright (c) 2013 Asger Nyman Christiansen. All rights reserved.
//

#include "normal_function.h"

void NormalFunc::deform(DeformableSimplicialComplex& dsc)
{
    clock_t init_time = clock();
    for(auto vi = dsc.vertices_begin(); vi != dsc.vertices_end(); ++vi)
    {
        if(dsc.is_movable(*vi))
        {
			std::vector<int> labels = dsc.get_interface_labels(*vi);
			if (labels[0]==0)
			{
				dsc.set_destination(*vi, dsc.get_pos(*vi) + VELOCITY * dsc.normal(*vi));
			}
        }
    }
    update_compute_time(clock() - init_time);
    init_time = clock();
    
    dsc.deform();
    
    update_deform_time(clock() - init_time);
}