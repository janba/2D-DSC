//
//  rotate_function.cpp
//  2D_DSC
//
//  Created by Asger Nyman Christiansen on 2/14/13.
//  Copyright (c) 2013 Asger Nyman Christiansen. All rights reserved.
//

#include "rotate_function.h"

#ifdef WIN32
#include <CGLA/Mat3x3f.h>
#else
#include <GEL/CGLA/Mat3x3f.h>
#endif


void RotateFunc::deform(DeformableSimplicialComplex& dsc)
{
    clock_t init_time = clock();
    
    CGLA::Vec2d cen = dsc.get_center();
    CGLA::Vec3f center = CGLA::Vec3f(cen[0], cen[1], 0.f);
    CGLA::Mat3x3f mrot = rotation_Mat3x3f(CGLA::ZAXIS, VELOCITY);
    CGLA::Vec2d p;
    CGLA::Vec3f new_pos;
    for(auto vi = dsc.vertices_begin(); vi != dsc.vertices_end(); ++vi)
    {
        if(dsc.is_movable(*vi))
        {
            p = dsc.get_pos(*vi);
            new_pos = center + mrot * (CGLA::Vec3f(p[0], p[1], 0.) - center);
            dsc.set_destination(*vi, CGLA::Vec2d(new_pos[0], new_pos[1]));
        }
    }
    update_compute_time(clock() - init_time);
    init_time = clock();
    
    dsc.deform();
    
    update_deform_time(clock() - init_time);
}