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

class Trializer {
    
    double WIDTH;
    double HEIGHT;
    double AVG_EDGE_LENGTH;
    
    int Nx, Ny;
    
    void create_points(std::vector<double>& points);
    
    void create_faces(std::vector<int>& faces);
    
public:
    
    Trializer(double width, double height, double avg_edge_length): WIDTH(width), HEIGHT(height), AVG_EDGE_LENGTH(avg_edge_length)
    {
        Nx = std::max(std::ceil(WIDTH/AVG_EDGE_LENGTH),1.);
        Ny = std::max(2*std::floor(HEIGHT/(sqrt(3.)*AVG_EDGE_LENGTH)),2.);
        
    }
    
    void trialize(std::vector<double>& points, std::vector<int>& faces)
    {
        create_points(points);
        create_faces(faces);
    }
};

