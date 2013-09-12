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

/**
 A domain class for specifying the design domain.
 */
class DesignDomain
{
    std::vector<CGLA::Vec2d> corners; // Specified in a clockwise order
    double volume = -1.;
    
public:
    enum DESIGN_DOMAIN_TYPE {RECTANGLE, L, ESO};
        
    DesignDomain(DESIGN_DOMAIN_TYPE design, int SIZE_X, int SIZE_Y, double boundary)
    {
        switch (design) {
            case RECTANGLE:
                corners.push_back(CGLA::Vec2d(0.,0.));
                corners.push_back(CGLA::Vec2d(0., SIZE_Y));
                corners.push_back(CGLA::Vec2d(SIZE_X, SIZE_Y));
                corners.push_back(CGLA::Vec2d(SIZE_X, 0.));
                break;
            case L:
                corners.push_back(CGLA::Vec2d(0.,0.));
                corners.push_back(CGLA::Vec2d(0., SIZE_Y));
                corners.push_back(CGLA::Vec2d(SIZE_X/2., SIZE_Y));
                corners.push_back(CGLA::Vec2d(SIZE_X/2., SIZE_Y/2.));
                corners.push_back(CGLA::Vec2d(SIZE_X, SIZE_Y/2.));
                corners.push_back(CGLA::Vec2d(SIZE_X, 0.));
                break;
            case ESO:
                corners.push_back(CGLA::Vec2d(0.,0.));
                corners.push_back(CGLA::Vec2d(0., 3.*SIZE_Y/7.));
                corners.push_back(CGLA::Vec2d(30.*SIZE_X/32., 3.*SIZE_Y/7.));
                corners.push_back(CGLA::Vec2d(30.*SIZE_X/32., SIZE_Y));
                corners.push_back(CGLA::Vec2d(31.*SIZE_X/32., SIZE_Y));
                corners.push_back(CGLA::Vec2d(31.*SIZE_X/32., 3.*SIZE_Y/7.));
                corners.push_back(CGLA::Vec2d(SIZE_X, 3.*SIZE_Y/7.));
                corners.push_back(CGLA::Vec2d(SIZE_X, 0.));
                break;
        }
        
        for(std::vector<CGLA::Vec2d>::iterator ci = corners.begin(); ci != corners.end(); ci++)
        {
            (*ci)[0] = (*ci)[0] + boundary;
            (*ci)[1] = (*ci)[1] + boundary;
        }
    }
    
    /**
     Creates a domain defined by the corners given as input. The corners should be specified in a clockwise order. It is possible to specify a boundary gap which translates the entire domain by the amount specified by the second input parameter. It is also possible to specify a label of the domain (used for easy creation of objects).
     */
    DesignDomain(std::vector<CGLA::Vec2d> _corners, double boundary = 0.): corners(_corners), volume(-1.)
    {
        for(std::vector<CGLA::Vec2d>::iterator ci = corners.begin(); ci != corners.end(); ci++)
        {
            (*ci)[0] = (*ci)[0] + boundary;
            (*ci)[1] = (*ci)[1] + boundary;
        }
    }
    
    /**
     Returns the corners of the design domain.
     */
    std::vector<CGLA::Vec2d> get_corners() const;
    
    /**
     Returns an approximate center of the design domain.
     */
    CGLA::Vec2d get_center();
    
    /**
     Returns the label associated with the domain.
     */
    int get_label();
    
    /**
     Returns the total volume of the domain.
     */
    double get_volume();
    
    /**
     Clamps the position pos to be within the domain.
     */
    void clamp_position(CGLA::Vec2d& p) const;
    
    /**
     Clamps p + v to be within the domain by scaling the vector v if p is inside the domain. The position p is therefore not garanteed to be within the domain.
     */
    void clamp_vector(const CGLA::Vec2d& p, CGLA::Vec2d& v) const;
    
    /**
     Returns whether the position p is inside the domain.
     */
    bool is_inside(CGLA::Vec2d p) const;
};
