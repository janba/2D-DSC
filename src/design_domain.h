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
    
    /**
     A domain class for specifying the design domain.
     */
    class DesignDomain
    {
        std::vector<vec2> corners; // Specified in a clockwise order
        std::vector<std::vector<vec2> > subdomain_corners;
        real volume = -1.;
        std::string name;
        double mWidth;
        double mHeight;
        
    public:
        enum DESIGN_DOMAIN_TYPE {RECTANGLE, L, ESO, PORTAL};
        
        /**
         Creates a design domain defined by the design domain type and size. It is possible to specify a boundary gap which translates the entire domain by the amount specified by the input parameter.
         */
        
        DesignDomain(DESIGN_DOMAIN_TYPE design, int SIZE_X, int SIZE_Y, real boundary1, real boundary2)
        {
            mWidth = SIZE_X;
            mHeight = SIZE_Y;
            switch (design)
            {
                case RECTANGLE:
                    name = "RECTANGLE";
                    corners.push_back(vec2(0.,0.));
                    corners.push_back(vec2(0., SIZE_Y));
                    corners.push_back(vec2(SIZE_X, SIZE_Y));
                    corners.push_back(vec2(SIZE_X, 0.));
                    subdomain_corners.push_back(corners);
                    break;
                case L:
                    name = "L";
                    corners.push_back(vec2(0.,0.));
                    corners.push_back(vec2(0., SIZE_Y));
                    corners.push_back(vec2(0.4*SIZE_X, SIZE_Y));
                    corners.push_back(vec2(0.4*SIZE_X, 0.4*SIZE_Y));
                    corners.push_back(vec2(SIZE_X, 0.4*SIZE_Y));
                    corners.push_back(vec2(SIZE_X, 0.));
                    subdomain_corners.push_back(corners);
                    break;
                case ESO:
                    name = "ESO";
                    corners.push_back(vec2(0.,0.));
                    corners.push_back(vec2(0., 3.*SIZE_Y/7.));
                    corners.push_back(vec2(30.*SIZE_X/32., 3.*SIZE_Y/7.));
                    corners.push_back(vec2(30.*SIZE_X/32., SIZE_Y));
                    corners.push_back(vec2(31.*SIZE_X/32., SIZE_Y));
                    corners.push_back(vec2(31.*SIZE_X/32., 3.*SIZE_Y/7.));
                    corners.push_back(vec2(SIZE_X, 3.*SIZE_Y/7.));
                    corners.push_back(vec2(SIZE_X, 0.));
                    subdomain_corners.push_back(corners);
                    break;
                case PORTAL:
                    name = "PORTAL";
                    corners.push_back(vec2(0.,0.));
                    corners.push_back(vec2(0., SIZE_Y));
                    corners.push_back(vec2(SIZE_X, SIZE_Y));
                    corners.push_back(vec2(SIZE_X, 0));
                    corners.push_back(vec2(0.95*SIZE_X, 0));
                    corners.push_back(vec2(0.5*SIZE_X, 0.65*SIZE_Y));
                    corners.push_back(vec2(0.05*SIZE_X, 0));
                    subdomain_corners.push_back({vec2(0.5*SIZE_X, 0.65*SIZE_Y), vec2(0.05*SIZE_X, 0), vec2(0.,0.), vec2(0., SIZE_Y), vec2(0.5*SIZE_X, SIZE_Y)});
                    subdomain_corners.push_back({ vec2(0.5*SIZE_X, 0.65*SIZE_Y), vec2(0.5*SIZE_X, SIZE_Y), vec2(SIZE_X, SIZE_Y), vec2(SIZE_X, 0.),  vec2(0.95*SIZE_X, 0) });
                    break;
            }
            
            for(auto &c : corners)
            {
                c[0] += boundary1;
                c[1] += boundary2;
            }
            
            for(auto &subdomain : subdomain_corners)
            {
                for(auto &sub_corner : subdomain)
                {
                    sub_corner[0] += boundary1;
                    sub_corner[1] += boundary2;
                }
            }
        }
        
        DesignDomain(std::string design, int SIZE_X, int SIZE_Y, real boundary1, real boundary2)
        {
            mWidth = SIZE_X;
            mHeight = SIZE_Y;
            if (design == "RECTANGLE")
            {
                name = "RECTANGLE";
                corners.push_back(vec2(0.,0.));
                corners.push_back(vec2(0., SIZE_Y));
                corners.push_back(vec2(SIZE_X, SIZE_Y));
                corners.push_back(vec2(SIZE_X, 0.));
                subdomain_corners.push_back(corners);
            }
            else if(design == "L")
            {
                name = "L";
                corners.push_back(vec2(0.,0.));
                corners.push_back(vec2(0., SIZE_Y));
                corners.push_back(vec2(0.4*SIZE_X, SIZE_Y));
                corners.push_back(vec2(0.4*SIZE_X, 0.4*SIZE_Y));
                corners.push_back(vec2(SIZE_X, 0.4*SIZE_Y));
                corners.push_back(vec2(SIZE_X, 0.));
                subdomain_corners.push_back(corners);
            }
            else if(design == "ESO")
            {
                name = "ESO";
                corners.push_back(vec2(0.,0.));
                corners.push_back(vec2(0., 3.*SIZE_Y/7.));
                corners.push_back(vec2(30.*SIZE_X/32., 3.*SIZE_Y/7.));
                corners.push_back(vec2(30.*SIZE_X/32., SIZE_Y));
                corners.push_back(vec2(31.*SIZE_X/32., SIZE_Y));
                corners.push_back(vec2(31.*SIZE_X/32., 3.*SIZE_Y/7.));
                corners.push_back(vec2(SIZE_X, 3.*SIZE_Y/7.));
                corners.push_back(vec2(SIZE_X, 0.));
                subdomain_corners.push_back(corners);
            }
            else if(design == "PORTAL")
            {
                name = "PORTAL";
                corners.push_back(vec2(0.,0.));
                corners.push_back(vec2(0., SIZE_Y));
                corners.push_back(vec2(SIZE_X, SIZE_Y));
                corners.push_back(vec2(SIZE_X, 0));
                corners.push_back(vec2(0.95*SIZE_X, 0));
                corners.push_back(vec2(0.5*SIZE_X, 0.65*SIZE_Y));
                corners.push_back(vec2(0.05*SIZE_X, 0));
                subdomain_corners.push_back({vec2(0.5*SIZE_X, 0.65*SIZE_Y), vec2(0.05*SIZE_X, 0), vec2(0.,0.), vec2(0., SIZE_Y), vec2(0.5*SIZE_X, SIZE_Y)});
                subdomain_corners.push_back({ vec2(0.5*SIZE_X, 0.65*SIZE_Y), vec2(0.5*SIZE_X, SIZE_Y), vec2(SIZE_X, SIZE_Y), vec2(SIZE_X, 0.),  vec2(0.95*SIZE_X, 0) });
            }
            else
            {
                std::cerr << "No available domain type!" << std::endl;
                std::exit(1);
            }
            
            for(auto &c : corners)
            {
                c[0] += boundary1;
                c[1] += boundary2;
            }
            
            for(auto &subdomain : subdomain_corners)
            {
                for(auto &sub_corner : subdomain)
                {
                    sub_corner[0] += boundary1;
                    sub_corner[1] += boundary2;
                }
            }
        }
        
        DesignDomain(DESIGN_DOMAIN_TYPE design, int SIZE_X, int SIZE_Y, real boundary) : DesignDomain(design, SIZE_X, SIZE_Y, boundary, boundary)
        {
            
        }

//        DesignDomain(DESIGN_DOMAIN_TYPE design, int SIZE_X, int SIZE_Y, real boundary)
//        {
//            mWidth = SIZE_X;
//            mHeight = SIZE_Y;
//            switch (design)
//            {
//                case RECTANGLE:
//                    name = "RECTANGLE";
//                    corners.push_back(vec2(0.,0.));
//                    corners.push_back(vec2(0., SIZE_Y));
//                    corners.push_back(vec2(SIZE_X, SIZE_Y));
//                    corners.push_back(vec2(SIZE_X, 0.));
//                    subdomain_corners.push_back(corners);
//                    break;
//                case L:
//                    name = "L";
//                    corners.push_back(vec2(0.,0.));
//                    corners.push_back(vec2(0., SIZE_Y));
//                    corners.push_back(vec2(0.4*SIZE_X, SIZE_Y));
//                    corners.push_back(vec2(0.4*SIZE_X, 0.4*SIZE_Y));
//                    corners.push_back(vec2(SIZE_X, 0.4*SIZE_Y));
//                    corners.push_back(vec2(SIZE_X, 0.));
//                    subdomain_corners.push_back(corners);
//                    break;
//                case ESO:
//                    name = "ESO";
//                    corners.push_back(vec2(0.,0.));
//                    corners.push_back(vec2(0., 3.*SIZE_Y/7.));
//                    corners.push_back(vec2(30.*SIZE_X/32., 3.*SIZE_Y/7.));
//                    corners.push_back(vec2(30.*SIZE_X/32., SIZE_Y));
//                    corners.push_back(vec2(31.*SIZE_X/32., SIZE_Y));
//                    corners.push_back(vec2(31.*SIZE_X/32., 3.*SIZE_Y/7.));
//                    corners.push_back(vec2(SIZE_X, 3.*SIZE_Y/7.));
//                    corners.push_back(vec2(SIZE_X, 0.));
//                    subdomain_corners.push_back(corners);
//                    break;
//                case PORTAL:
//                    name = "PORTAL";
//                    corners.push_back(vec2(0.,0.));
//                    corners.push_back(vec2(0., SIZE_Y));
//                    corners.push_back(vec2(SIZE_X, SIZE_Y));
//                    corners.push_back(vec2(SIZE_X, 0));
//                    corners.push_back(vec2(0.95*SIZE_X, 0));
//                    corners.push_back(vec2(0.5*SIZE_X, 0.65*SIZE_Y));
//                    corners.push_back(vec2(0.05*SIZE_X, 0));
//                    subdomain_corners.push_back({vec2(0.5*SIZE_X, 0.65*SIZE_Y), vec2(0.05*SIZE_X, 0), vec2(0.,0.), vec2(0., SIZE_Y), vec2(0.5*SIZE_X, SIZE_Y)});
//                    subdomain_corners.push_back({ vec2(0.5*SIZE_X, 0.65*SIZE_Y), vec2(0.5*SIZE_X, SIZE_Y), vec2(SIZE_X, SIZE_Y), vec2(SIZE_X, 0.),  vec2(0.95*SIZE_X, 0) });
//                    break;
//            }
//            
//            for(auto &c : corners)
//            {
//                c[0] += boundary;
//                c[1] += boundary;
//            }
//            
//            for(auto &subdomain : subdomain_corners)
//            {
//                for(auto &sub_corner : subdomain)
//                {
//                    sub_corner[0] += boundary;
//                    sub_corner[1] += boundary;
//                }
//            }
//        }

        DesignDomain(std::string design, int SIZE_X, int SIZE_Y, real boundary) : DesignDomain(design, SIZE_X, SIZE_Y, boundary, boundary)
        {
            
        }
//        DesignDomain(std::string design, int SIZE_X, int SIZE_Y, real boundary)
//        {
//            mWidth = SIZE_X;
//            mHeight = SIZE_Y;
//            if (design == "RECTANGLE")
//            {
//                name = "RECTANGLE";
//                corners.push_back(vec2(0.,0.));
//                corners.push_back(vec2(0., SIZE_Y));
//                corners.push_back(vec2(SIZE_X, SIZE_Y));
//                corners.push_back(vec2(SIZE_X, 0.));
//                subdomain_corners.push_back(corners);
//            }
//            else if(design == "L")
//            {
//                name = "L";
//                corners.push_back(vec2(0.,0.));
//                corners.push_back(vec2(0., SIZE_Y));
//                corners.push_back(vec2(0.4*SIZE_X, SIZE_Y));
//                corners.push_back(vec2(0.4*SIZE_X, 0.4*SIZE_Y));
//                corners.push_back(vec2(SIZE_X, 0.4*SIZE_Y));
//                corners.push_back(vec2(SIZE_X, 0.));
//                subdomain_corners.push_back(corners);
//            }
//            else if(design == "ESO")
//            {
//                name = "ESO";
//                corners.push_back(vec2(0.,0.));
//                corners.push_back(vec2(0., 3.*SIZE_Y/7.));
//                corners.push_back(vec2(30.*SIZE_X/32., 3.*SIZE_Y/7.));
//                corners.push_back(vec2(30.*SIZE_X/32., SIZE_Y));
//                corners.push_back(vec2(31.*SIZE_X/32., SIZE_Y));
//                corners.push_back(vec2(31.*SIZE_X/32., 3.*SIZE_Y/7.));
//                corners.push_back(vec2(SIZE_X, 3.*SIZE_Y/7.));
//                corners.push_back(vec2(SIZE_X, 0.));
//                subdomain_corners.push_back(corners);
//            }
//            else if(design == "PORTAL")
//            {
//                name = "PORTAL";
//                corners.push_back(vec2(0.,0.));
//                corners.push_back(vec2(0., SIZE_Y));
//                corners.push_back(vec2(SIZE_X, SIZE_Y));
//                corners.push_back(vec2(SIZE_X, 0));
//                corners.push_back(vec2(0.95*SIZE_X, 0));
//                corners.push_back(vec2(0.5*SIZE_X, 0.65*SIZE_Y));
//                corners.push_back(vec2(0.05*SIZE_X, 0));
//                subdomain_corners.push_back({vec2(0.5*SIZE_X, 0.65*SIZE_Y), vec2(0.05*SIZE_X, 0), vec2(0.,0.), vec2(0., SIZE_Y), vec2(0.5*SIZE_X, SIZE_Y)});
//                subdomain_corners.push_back({ vec2(0.5*SIZE_X, 0.65*SIZE_Y), vec2(0.5*SIZE_X, SIZE_Y), vec2(SIZE_X, SIZE_Y), vec2(SIZE_X, 0.),  vec2(0.95*SIZE_X, 0) });
//            }
//            else
//            {
//                std::cerr << "No available domain type!" << std::endl;
//                std::exit(1);
//            }
//            
//            for(auto &c : corners)
//            {
//                c[0] += boundary;
//                c[1] += boundary;
//            }
//            
//            for(auto &subdomain : subdomain_corners)
//            {
//                for(auto &sub_corner : subdomain)
//                {
//                    sub_corner[0] += boundary;
//                    sub_corner[1] += boundary;
//                }
//            }
//        }
        
        DesignDomain()
        {
            
        }
        
        std::string get_name() const
        {
            return name;
        }
        
        double get_width() const
        {
            return mWidth;
        }
        
        double get_height() const
        {
            return mHeight;
        }
        
        void set_corners(std::vector<vec2> input_corners)
        {
            for (auto corner : input_corners )
            {
                corners.push_back(corner);
            }
        }
        
        void add_corner(vec2 corner)
        {
            corners.push_back(corner);
        }
        
        /**
         Returns the corners of the design domain.
         */
        std::vector<vec2> get_corners() const;
        
        std::vector<std::vector<vec2> > get_subdomain_corners() const;
        
        /**
         Returns an approximate center of the design domain.
         */
        vec2 get_center() const;
        
        /**
         Returns the total volume of the domain.
         */
        real get_volume();
        
        real get_perimeter();
        
        /**
         Clamps the position pos to be within the domain.
         */
        void clamp_position(vec2& p) const;
        
        /**
         Clamps p + v to be within the domain by scaling the vector v if p is inside the domain. The position p is therefore not garanteed to be within the domain.
         */
        void clamp_vector(const vec2& p, vec2& v) const;
        
        /**
         Returns whether the position p is inside the domain.
         */
        bool is_inside(const vec2& p) const;
        
        /**
         Returns whether the position p is on the boundary.
         */
        bool on_boundary(const vec2& p) const;
    };
    
}