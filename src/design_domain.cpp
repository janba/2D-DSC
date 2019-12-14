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

#include "design_domain.h"

namespace DSC2D {
    
    void DesignDomain::clamp_position(vec2& p) const
    {
        if(!is_inside(p))
        {
            vec2 c0, c1, b, p_proj, p_int1, p_int2;
            real length, dist1 = INFINITY, dist2 = INFINITY;
            for (int i = 0; i < corners.size(); i++)
            {
                c0 = corners[i];
                c1 = corners[(i+1)%corners.size()];
                b = Util::normalize(c1 - c0);
                p_proj = c0 + Util::dot(p - c0, b)*b;
                length = Util::sqr_length(p - p_proj);
                if (length < dist1)
                {
                    p_int2 = p_int1;
                    p_int1 = p_proj;
                    dist2 = dist1;
                    dist1 = length;
                }
                else if(length < dist2)
                {
                    p_int2 = p_proj;
                    dist2 = length;
                }
            }
            if(!is_inside(p_int1))
            {
                p = p_int1 + (p_int2 - p);
            }
            else {
                p = p_int1;
            }
        }
    }
    
    void DesignDomain::clamp_vector(const vec2& p, vec2& v) const
    {
//        if (!is_inside(p))
//        {
//            vec2 pc = p;
//            clamp_position(pc);
//    
//            real distance_now = sqr_length(p - pc);
//            real distance_future = sqr_length(p + v - pc);
//            
//            if (distance_future > distance_now)
//            {
//                v = vec2(0, 0);
//            }
//            return;
//        }
        
        if(!is_inside(p+v))
        {
            vec2 c0, c1;
            real t;
            for (int i = 0; i < corners.size(); i++)
            {
                c0 = corners[i];
                c1 = corners[(i+1)%corners.size()];
                
                // Haojie Added
                real x = Util::cross(c0-p, c1-c0);
                real y = Util::cross(v, c1-c0);
                if(std::abs(y) < EPSILON) // v and the segment are parallel if true
                {
                    if(std::abs(x) < EPSILON) // v and the segment are collinear if true
                    {
                        if ( dot(c0-p, c1-p) <= 0 )
                        {
                            if( (p+v-c1).length() <= (p+v-c0).length())
                            {
                                v = c1 - p;
                            }
                            else
                            {
                                v = c0 - p;
                            }
                            
                            return;
                        }
                    }
                }
                // Haojie finished the added code segement.
                
                t = Util::intersection_ray_ray(p, v, c0, c1 - c0);
                // if(t >= 0. && t < 1.)
                if ( ( t>=0 && t < 1. ) || std::abs(t)< EPSILON )
                {
                    v = t*v;
                }
            }
        }
    }
    
    bool DesignDomain::is_inside(const vec2& p) const
    {
        return Util::is_inside(p, corners);
    }
    
    bool DesignDomain::on_boundary(const vec2& p) const
    {
        return Util::on_boundary(p, corners);
    }
    
    std::vector<vec2> DesignDomain::get_corners() const
    {
        return corners;
    }
    
    std::vector<std::vector<vec2> > DesignDomain::get_subdomain_corners() const
    {
        return subdomain_corners;
    }
    
    vec2 DesignDomain::get_center() const
    {
        vec2 center(0.);
        for (int i = 0; i < corners.size(); i++)
        {
            center += corners[i];
        }
        return center/static_cast<real>(corners.size());
    }
    
    real DesignDomain::get_volume()
    {
//        if(volume < 0.)
//        {
//            volume = 0.;
//            std::vector<vec2> c(corners);
//            vec2 c0, c1, c2;
//            while(c.size() > 2)
//            {
//                int i = 0;
//                do {
//#ifdef DEBUG
//                    assert(i < c.size());
//#endif
//                    c0 = c[i];
//                    c1 = c[(i+1)%c.size()];
//                    c2 = c[(i+2)%c.size()];
//                    i++;
//                } while (Util::is_left_of(c0,c1,c2));
//                
//                volume += std::abs(Util::signed_area(c0, c1, c2));
//                c.erase(c.begin() + (i%c.size()));
//            }
//        }
        
        if(volume < 0.)
        {
            std::vector<vec2> c(corners);
            c.push_back(corners[0]);
            
            double sum1 = 0;
            double sum2 = 0;
            
            for (int i=0; i<c.size()-1; i++)
            {
                sum1 += c[i][0] * c[i+1][1];
                sum2 += c[i][1] * c[i+1][0];
            }
            
            volume = std::abs(0.5 * (sum1 - sum2));
        }
       
        return volume;
    }
    
    real DesignDomain::get_perimeter()
    {
        std::vector<vec2> c(corners);
        c.push_back(corners[0]);
        
        double perimeter = 0;
        
        for (int i=0; i<c.size()-1; i++)
        {
            perimeter += (c[i] - c[i+1]).length();
        }
        
        return perimeter;
    }
    
}