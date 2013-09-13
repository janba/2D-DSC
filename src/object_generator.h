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

#pragma once

#include "util.h"
#include "DSC.h"

namespace DSC2D {
    
    class ObjectGenerator {
        
        static void fit_mesh_to_object(DeformableSimplicialComplex& dsc, const std::vector<vec2>& corners);
        
        static void label_faces(DeformableSimplicialComplex& dsc, const std::vector<vec2>& corners, int label);
        
        static void create_object(DeformableSimplicialComplex& dsc, const std::vector<vec2>& corners, int label);
        
    public:
        
        static void create_blob(DeformableSimplicialComplex& dsc, const vec2& center, const double& radius, int label)
        {
            std::vector<vec2> corners;
            for (double a = 0; a < 2*M_PI; a += (1./32.)*2*M_PI)
            {
                corners.push_back(radius*vec2(std::cos(a), -std::sin(a)) + center);
            }
            create_object(dsc, corners, label);
        }
        
        static void create_square(DeformableSimplicialComplex& dsc, const vec2& origin, const vec2& size, int label)
        {
            std::vector<vec2> corners;
            corners.push_back(origin);
            corners.push_back(origin + vec2(0., size[1]));
            corners.push_back(origin + size);
            corners.push_back(origin + vec2(size[0], 0.));
            
            create_object(dsc, corners, label);
        }
    };
    
    
}