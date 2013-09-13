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

namespace DSC2D {
    
    class Trializer {
        
        static void create_points(int Nx, int Ny, double width, double height, double avg_edge_length, std::vector<double>& points);
        
        static void create_faces(int Nx, int Ny, std::vector<int>& faces);
        
    public:
        
        static void trialize(double width, double height, double avg_edge_length, std::vector<double>& points, std::vector<int>& faces)
        {
            int Nx = std::max(std::ceil(width/avg_edge_length),1.);
            int Ny = std::max(2*std::floor(height/(sqrt(3.)*avg_edge_length)),2.);
            
            create_points(Nx, Ny, width, height, avg_edge_length, points);
            create_faces(Nx, Ny, faces);
        }
    };
    
}