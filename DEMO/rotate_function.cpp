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

#include "rotate_function.h"

#ifdef WIN32
#include <CGLA/Mat3x3f.h>
#else
#include <GEL/CGLA/Mat3x3f.h>
#endif


void RotateFunc::deform(DSC2D::DeformableSimplicialComplex& dsc)
{
    auto init_time = std::chrono::system_clock::now();
    
    DSC2D::vec2 cen = dsc.get_center();
    CGLA::Vec3f center = CGLA::Vec3f(cen[0], cen[1], 0.f);
    CGLA::Mat3x3f mrot = rotation_Mat3x3f(CGLA::ZAXIS, VELOCITY);
    DSC2D::vec2 p;
    CGLA::Vec3f destination;
    for(auto vi = dsc.vertices_begin(); vi != dsc.vertices_end(); ++vi)
    {
        if(dsc.is_movable(*vi))
        {
            p = dsc.get_pos(*vi);
            destination = center + mrot * (CGLA::Vec3f(p[0], p[1], 0.) - center);
            dsc.set_destination(*vi, DSC2D::vec2(destination[0], destination[1]));
        }
    }
    update_compute_time(init_time);
    init_time = std::chrono::system_clock::now();
    
    dsc.deform();
    
    update_deform_time(init_time);
}