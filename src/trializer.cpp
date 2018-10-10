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

#include "trializer.h"

namespace DSC2D {
    
    void Trializer::create_points(int Nx, int Ny, real width, real height, real avg_edge_length, std::vector<real>& points)
    {
        real b = (real)width/(real)Nx; // adjusted length of the triangle base
        real h = (real)height/(real)Ny; // adjusted triangle height
        
        // x positions of points on triangle basis
        std::vector<real> x_full;
        std::vector<real> x_half;
        
        x_full.push_back(0.);
        for (real x=avg_edge_length; x<avg_edge_length+(Nx+1)*b; x+=b) // stopping condition: x<= AVG_EDGE_LENGTH+Nx*b
        {
            x_full.push_back(x);
        }
        x_full.push_back(width+2*avg_edge_length);
        
        // x positions of points on triangle tops/bottoms
        x_half.push_back(0.); x_half.push_back(avg_edge_length);
        for (real x=avg_edge_length+b/2; x<avg_edge_length+(Nx+1)*b-b/2; x+=b) // stopping condition: x<=AVG_EDGE_LENGTH+Nx*b-b/2
        {
            x_half.push_back(x);
        }
        x_half.push_back(width+avg_edge_length); x_half.push_back(width+2*avg_edge_length);
        
        for (int k = 0; k<x_full.size(); k++)
        {
            points.push_back(x_full[k]); points.push_back(0); points.push_back(0); // first line of points
        }
        
        for (real y = avg_edge_length; y<avg_edge_length + Ny*h; y+=2*h) // stopping condition: y<=AVG_EDGE_LENGTH + (Ny-2)*h;
        {
            for (int k = 0; k<x_full.size(); k++)
            {
                points.push_back(x_full[k]); points.push_back(y); points.push_back(0); // lines with points on triangle basis
            }
            for (int k = 0; k<x_half.size(); k++)
            {
                points.push_back(x_half[k]); points.push_back(y+h); points.push_back(0); // lines with points on triangle tops/bottoms
            }
        }
        for (int k = 0; k<x_full.size(); k++)
        {
            points.push_back(x_full[k]); points.push_back(height+avg_edge_length); points.push_back(0); // line between inside and boundary
        }
        for (int k = 0; k<x_full.size(); k++)
        {
            points.push_back(x_full[k]); points.push_back(height+2*avg_edge_length); points.push_back(0); // last line of points
        }
    }
    
    void Trializer::create_faces(int Nx, int Ny, std::vector<int>& faces)
    {
        // boundary
        std::vector<int> f_b0(6,0), f_b1(6,0);
        f_b0[0]=0; f_b0[1]=1; f_b0[2]=Nx+4; f_b0[3]=0; f_b0[4]=Nx+4; f_b0[5]=Nx+3; // line skip Nx+3
        f_b1[0]=0; f_b1[1]=1; f_b1[2]=Nx+5; f_b1[3]=0; f_b1[4]=Nx+5; f_b1[5]=Nx+4; // line skip Nx+4
        
        for (int i = 0; i<Nx+2; i++) // stopping condition: i<=Nx+1
        {
            for (int k = 0; k<f_b0.size(); k++)
            {
                faces.push_back(f_b0[k]+i); // boundary bottom
            }
            for (int k = 0; k<f_b0.size(); k++)
            {
                faces.push_back(f_b0[k]+(Nx+3)*(1+Ny)+Ny/2+i); // boundary top
            }
        }
        
        for (int j = 0; j<Ny*(2*Nx+7)/2; j+=2*Nx+7) // stopping condition: j<=(Ny-2)*(2*Nx+7)/2
        {
            for (int k = 0; k<f_b0.size(); k++)
            {
                faces.push_back(f_b0[k]+Nx+3+j); // boundary left
            }
            for (int k = 0; k<f_b1.size(); k++)
            {
                faces.push_back(f_b1[k]+2*Nx+4+j); // boundary right
            }
            for (int k = 0; k<f_b1.size(); k++)
            {
                faces.push_back(f_b1[k]+2*Nx+6+j); // boundary left
            }
            for (int k = 0; k<f_b0.size(); k++)
            {
                faces.push_back(f_b0[k]+3*Nx+8+j); // boundary right
            }
        }
        
        // inside
        std::vector<int> f_down(3,0), f_up(3,0);
        f_down[0]=0; f_down[1]=1; f_down[2]=Nx+4; // triangles with base down
        f_up[0]=0; f_up[1]=Nx+4; f_up[2]=Nx+3; // triangles with base down
        
        for (int j = 0; j<(Ny/2)*(2*Nx+7); j+=2*Nx+7) // stopping condition: j<=(Ny/2-1)*(2*Nx+7)
        {
            for (int i = 0; i<Nx; i++) // stopping condition: i<=Nx-1
            {
                for (int k = 0; k<f_down.size(); k++)
                {
                    faces.push_back(f_down[k]+Nx+4+i+j);
                }
                for (int k = 0; k<f_up.size(); k++)
                {
                    faces.push_back(f_up[k]+2*Nx+8+i+j);
                }
            }
            for (int i = 0; i<Nx+1; i++) // stopping condition: i<=Nx
            {
                for (int k = 0; k<f_up.size(); k++)
                {
                    faces.push_back(f_up[k]+Nx+4+i+j);
                }
                for (int k = 0; k<f_down.size(); k++)
                {
                    faces.push_back(f_down[k]+2*Nx+7+i+j);
                }
            }
        }
    }
    
}