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

class ObjectGenerator {
    
    static void fit_mesh_to_object(DeformableSimplicialComplex& dsc, const std::vector<CGLA::Vec2d>& corners)
    {
        // Move a vertex to all corners
        for (auto fi = dsc.faces_begin(); fi != dsc.faces_end(); fi++)
        {
            auto verts = dsc.get_pos(*fi);
            
            for (auto & c : corners) {
                if(Util::is_inside(c, verts))
                {
                    HMesh::VertexID vid;
                    double min_dist = INFINITY;
                    for (HMesh::Walker hew = dsc.walker(*fi); !hew.full_circle(); hew = hew.circulate_face_cw())
                    {
                        double l = (dsc.get_pos(hew.vertex()) - c).length();
                        if(l < min_dist)
                        {
                            min_dist = l;
                            vid = hew.vertex();
                        }
                    }
                    dsc.set_pos(vid, c);
                }
            }
        }
        
        // Make sure no face is both inside and outside the object
        for (auto fi = dsc.faces_begin(); fi != dsc.faces_end(); fi++)
        {
            auto verts = dsc.get_pos(*fi);
            std::vector<bool> inside;
            for (auto &v : verts) {
                inside.push_back(Util::is_inside(v, corners));
            }
            int sum = inside[0] + inside[1] + inside[2];
            
            if(sum == 1 || sum == 2) // Face is on the interface of the object and two vertices are inside.
            {
                int i = 0;
                for (HMesh::Walker hew = dsc.walker(*fi); !hew.full_circle(); hew = hew.circulate_face_cw(), i++)
                {
                    if((sum == 1 && inside[i]) || (sum == 2 && !inside[i])) // Move the vertex
                    {
                        // Find intersection point
                        CGLA::Vec2d point;
                        CGLA::Vec2d p = dsc.get_pos(hew.opp().vertex());
                        CGLA::Vec2d r = dsc.get_pos(hew.vertex()) - p;
                        CGLA::Vec2d q, s;
                        for(int j = 0; j < corners.size(); j++)
                        {
                            q = corners[j];
                            s = corners[(j+1)%corners.size()] - q;
                            double t = Util::intersection(p, r, q, s);
                            if(0. <= t && t <= 1.)
                            {
                                point = p + t*r;
                                break;
                            }
                        }
                        
                        if((point - p).length() < (point - (p + r)).length())
                        {
                            dsc.set_pos(hew.opp().vertex(), point);
                        }
                        else {
                            dsc.set_pos(hew.vertex(), point);
                        }
                    }
                }
            }
        }
    }
    
    static void label_faces(DeformableSimplicialComplex& dsc, const std::vector<CGLA::Vec2d>& corners, int label)
    {
        for (auto fi = dsc.faces_begin(); fi != dsc.faces_end(); fi++)
        {
            auto verts = dsc.get_pos(*fi);
            
            std::vector<bool> inside;
            for (auto &v : verts) {
                inside.push_back(Util::is_inside(v, corners));
            }
            
            if(inside[0] && inside[1] && inside[2]) // Face is inside the object.
            {
                dsc.face_labels[*fi] = label;
            }
        }
    }
    
    static void create_object(DeformableSimplicialComplex& dsc, const std::vector<CGLA::Vec2d>& corners, int label)
    {
        fit_mesh_to_object(dsc, corners);
        label_faces(dsc, corners, label);
        
        dsc.init_attributes();
        dsc.update_attributes();
        dsc.fix_mesh();
    }
    
public:
    
    static void create_blob(DeformableSimplicialComplex& dsc, const CGLA::Vec2d& center, const double& radius, int label)
    {
        std::vector<CGLA::Vec2d> corners;
        for (double a = 0; a < 2*M_PI; a += (1./32.)*2*M_PI)
        {
            corners.push_back(radius*CGLA::Vec2d(std::cos(a), -std::sin(a)) + center);
        }
        create_object(dsc, corners, label);
    }
    
    static void create_square(DeformableSimplicialComplex& dsc, const CGLA::Vec2d& origin, const CGLA::Vec2d& size, int label)
    {
        std::vector<CGLA::Vec2d> corners;
        corners.push_back(origin);
        corners.push_back(origin + CGLA::Vec2d(0., size[1]));
        corners.push_back(origin + size);
        corners.push_back(origin + CGLA::Vec2d(size[0], 0.));
        
        create_object(dsc, corners, label);
    }
};

