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

#include <vector>
#include <cmath>
#include <sstream>

#ifdef WIN32
#include <CGLA/Vec2d.h>
#include <CGLA/Vec3d.h>
#else
#include <GEL/CGLA/Vec2d.h>
#include <GEL/CGLA/Vec3d.h>
#endif


namespace DSC2D
{
    typedef double real;
    typedef CGLA::Vec2d vec2;
    typedef CGLA::Vec3d vec3;
    
    constexpr real EPSILON = 1e-8;
    
#ifndef INFINITY
    constexpr real INFINITY = 1e32;
#endif
    
    namespace Util
    {
        
        inline real min(real x, real y)
        {
            return std::min(x, y);
        }
        
        inline real max(real x, real y)
        {
            return std::max(x, y);
        }
        
        /**
         Computes the cosine to the angle at point p1.
         */
        inline real cos_angle(const vec2& p0,const vec2& p1, const vec2& p2)
        {
            vec2 a = p2-p1;
            vec2 b = p0-p1;
            
            real sqla = sqr_length(a);
            if(sqla < EPSILON)
            {
                return 0.0;
            }
            a/= sqrt(sqla);
            
            
            real sqlb = sqr_length(b);
            if(sqlb < EPSILON)
            {
                return 0.0;
            }
            b/= sqrt(sqlb);
            
            real ang = max(-1., min(1., dot(a,b)));
#ifdef DEBUG
            assert(!CGLA::isnan(ang));
#endif
            return ang;
        }
        
        /**
         Computes the minimal angle of a triangle.
         */
        inline real min_angle(const vec2& p0,const vec2& p1,const vec2& p2)
        {
            real a1 = acos(cos_angle(p2, p0, p1));
            real a2 = acos(cos_angle(p0, p1, p2));
            real a3 = acos(cos_angle(p1, p2, p0));
            real m = min(a1,min(a2,a3));
#ifdef DEBUG
            assert(std::abs(a1 + a2 + a3 - M_PI) < 0.001 );
            assert(!CGLA::isnan(m));
#endif
            return m;
        }
        
        
        /**
         Computes the signed area of a triangle.
         */
        inline real signed_area(const vec2& v0,const vec2& v1,const vec2& v2)
        {
            real A = 0.5*CGLA::cross(v1-v0,v2-v0);
#ifdef DEBUG
            assert(!CGLA::isnan(A));
#endif
            return A;
        }
        
        /**
         Computes the area of a triangle.
         */
        inline real area(const vec2& v0,const vec2& v1,const vec2& v2)
        {
            return std::abs(signed_area(v0, v1, v2));
        }
        
        /**
         Returns the vector a projected onto the vector b.
         */
        inline vec2 project(const vec2& a,const vec2& b)
        {
            return b * dot(a,b)/dot(b, b);
        }
        
        /**
         Returns the squared minimum distance between line segment vw and point p.
         */
        inline real min_dist_sqr(const vec2& v, const vec2& w, const vec2& p)
        {
            const real l2 = dot(v-w, v-w);  // i.e. |w-v|^2 -  avoid a sqrt
            if (l2 == 0.)
            {
                return sqr_length(p - v);   // v == w case
            }
            // Consider the line extending the segment, parameterized as v + t (w - v).
            // We find projection of point p onto the line.
            // It falls where t = [(p-v) . (w-v)] / |w-v|^2
            const real t = dot(p - v, w - v) / l2;
            if (t < 0.0)
            {
                return sqr_length(p - v);       // Beyond the 'v' end of the segment
            }
            else if (t > 1.0)
            {
                return sqr_length(p - w);  // Beyond the 'w' end of the segment
            }
            const vec2 projection = v + t * (w - v);  // Projection falls on the segment
            return sqr_length(p - projection);
        }
        
        /**
         Returns whether you have to turn left when going from a to b to c.
         */
        inline bool is_left_of(vec2 a, vec2 b, vec2 c)
        {
            if(signed_area(a,b,c) > 0.)
            {
                return true;
            }
            return false;
        }
        
        /**
         Returns whether the point c is between point a and b.
         */
        inline bool is_between(vec2 a, vec2 b, vec2 c)
        {
            if(cross(a-b,c-b) > 0.)
            {
                return false;
            }
            real l = (a-b).length();
            if ((a-c).length() <= l && (b-c).length() <= l)
            {
                return true;
            }
            return false;
        }
        
        /**
         Returns whether the point p is between the points in the vector corners.
         */
        inline bool is_inside(vec2 p, std::vector<vec2> corners)
        {
            vec2 c0, c1, c2;
            while(corners.size() > 2)
            {
                int i = 0;
                do {
#ifdef DEBUG
                    assert(i < corners.size());
#endif
                    c0 = corners[i];
                    c1 = corners[(i+1)%corners.size()];
                    c2 = corners[(i+2)%corners.size()];
                    i++;
                } while (is_left_of(c0,c1,c2));
                
                if(!is_left_of(c0, c1, p) && !is_left_of(c1, c2, p) && !is_left_of(c2, c0, p))
                {
                    return true;
                }
                corners.erase(corners.begin() + (i%corners.size()));
            }
            return false;
        }
        
        /**
         Calculates the intersection between the line segments defined by p + t*r and q + u*s, where 0 <= t <= 1 and 0 <= u <= 1. The method returns t if 0 <= u <= 1 and INFINITY otherwise. If t is not in the range 0 <= t <= 1, the line segments do not intersect.
         */
        inline real intersection(vec2 p, vec2 r, vec2 q, vec2 s)
        {
            real t = INFINITY;
            real a = cross(q-p, s);
            real b = cross(r,s);
            if(std::abs(b) < EPSILON) // r and s are parallel if true
            {
                if(std::abs(a) < EPSILON) // r and s are collinear if true
                {
                    if (is_between(q, q+s, p) || is_between(q, q+s, p+r) || is_between(p, p+r, q))
                    {
                        t = 0.;
                    }
                }
            }
            else
            {
                real u = cross(q-p, r)/b;
                if (0. <= u && u <= 1.)
                {
                    t = a/b;
                }
            }
            return t;
        }
        
        /**
         Calculates the jet color from the value.
         */
        inline vec3 jet_color(real value)
        {
            real fourValue = 4 * value;
            real red   = min(fourValue - 1.5, -fourValue + 4.5);
            real green = min(fourValue - 0.5, -fourValue + 3.5);
            real blue  = min(fourValue + 0.5, -fourValue + 2.5);
            vec3 col(red, green, blue);
            for (int i = 0; i < 3; i++)
            {
                col[i] = max(min(col[i] , 1.), 0.);
            }
            return col;
        }
        
        inline vec3 color(vec3 start_color, int i)
        {
            if (i <= 0) {
                return start_color;
            }
            i++;
            real step = static_cast<real>(i - i%3)/3.;
            vec3 col = start_color;
            col[i%3] += 0.7*step;
            col[i%3] -= std::floor(col[i%3]);
            return col;
        }
        
        
        /**
         Returns the sign of the input val.
         */
        template <typename T>
        inline int sign(T val) {
            return (T(0) < val) - (val < T(0));
        }
        
        
        /**
         Returns the maximum difference between the values x[i] and y[i] for all i less than the size of x and y.
         */
        inline real max_diff(const std::vector<real>& x, const std::vector<real>& y)
        {
            real temp, diff = -INFINITY;
            for (int i = 0; i < x.size(); i++)
            {
                if (i < y.size()) {
                    temp = std::abs(x[i] - y[i]);
                    if (temp > diff) {
                        diff = temp;
                    }
                }
            }
            return diff;
        }
        
        /**
         Concatonates the integer number to the string name.
         */
        inline std::string concat4digits(std::string name, int number)
        {
            std::ostringstream s;
            if (number < 10)
                s << name << "000" << number;
            else if (number < 100)
                s << name << "00" << number;
            else if (number < 1000)
                s << name << "0" << number;
            else
                s << name << number;
            return s.str();
        }
        
    }
}
