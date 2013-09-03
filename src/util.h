//
//  util.h
//  2D_DSC
//
//  Created by Asger Nyman Christiansen on 3/23/12.
//  Copyright (c) 2012 DTU Informatics. All rights reserved.
//

#ifndef _DDSC_util_h
#define _DDSC_util_h

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

const double EPSILON = 1e-8;

#ifndef INFINITY
const double INFINITY = 1e32;
#endif


namespace Util
{
        
    /**
     Computes the cosine to the angle at point p1.
     */
    template <class vec>
    inline double cos_angle(const vec& p0,const vec& p1, const vec& p2)
    {
        vec a = p2-p1;
        vec b = p0-p1;
        
        double sqla = sqr_length(a);
        if(sqla < EPSILON)
        {
            return 0.0;
        }
        a/= sqrt(sqla);
        
        
        double sqlb = sqr_length(b);
        if(sqlb < EPSILON)
        {
            return 0.0;
        }
        b/= sqrt(sqlb);
        
        double ang = std::max(-1.0, std::min(1.0, double(dot(a,b))));
#ifdef DEBUG
        assert(!CGLA::isnan(ang));
#endif
        return ang;
    }
    
    /**
     Computes the minimal angle of a triangle.
     */
    template <class vec>
    inline double min_angle(const vec& p0,const vec& p1,const vec& p2)
    {
        double a1 = acos(cos_angle(p2, p0, p1));
        double a2 = acos(cos_angle(p0, p1, p2));
        double a3 = acos(cos_angle(p1, p2, p0));
        double m = std::min(a1,std::min(a2,a3));
#ifdef DEBUG
        assert(std::abs(a1 + a2 + a3 - M_PI) < 0.001 );
        assert(!CGLA::isnan(m));
#endif
        return m;
    }
    
    
    /**
     Computes the signed area of a triangle.
     */
    template <class vec>
    inline double signed_area(const vec& v0,const vec& v1,const vec& v2)
    {
        double A = 0.5*CGLA::cross(v1-v0,v2-v0);
#ifdef DEBUG
        assert(!CGLA::isnan(A));
#endif
        return A;
    }
    
    /**
     Computes the area of a triangle.
     */
    template <class vec>
    inline double area(const vec& v0,const vec& v1,const vec& v2)
    {
        return std::abs(signed_area(v0, v1, v2));
    }
    
    /**
     Returns the vector a projected onto the vector b.
     */
    template <class vec>
    inline vec project(const vec& a,const vec& b)
    {
        return b * dot(a,b)/dot(b, b);
    }
    
    /**
     Returns the minimum distance between line segment vw and point p.
     */
    template <class vec>
    inline double min_dist(const vec& v, const vec& w, const vec& p)
    {
        const double l2 = dot(v-w, v-w);  // i.e. |w-v|^2 -  avoid a sqrt
        if (l2 == 0.)
        {
            return (p - v).length();   // v == w case
        }
        // Consider the line extending the segment, parameterized as v + t (w - v).
        // We find projection of point p onto the line.
        // It falls where t = [(p-v) . (w-v)] / |w-v|^2
        const double t = dot(p - v, w - v) / l2;
        if (t < 0.0)
        {
            return (p - v).length();       // Beyond the 'v' end of the segment
        }
        else if (t > 1.0)
        {
            return (p - w).length();  // Beyond the 'w' end of the segment
        }
        const vec projection = v + t * (w - v);  // Projection falls on the segment
        return (p - projection).length();
    }
    
    /**
     Returns whether you have to turn left when going from a to b to c.
     */
    inline bool is_left_of(CGLA::Vec2d a, CGLA::Vec2d b, CGLA::Vec2d c)
    {
        if(Util::signed_area<CGLA::Vec2d>(a,b,c) > 0.)
        {
            return true;
        }
        return false;
    }
    
    /**
     Returns whether the point c is between point a and b.
     */
    inline bool is_between(CGLA::Vec2d a, CGLA::Vec2d b, CGLA::Vec2d c)
    {
        if(cross(a-b,c-b) > 0.)
        {
            return false;
        }
        double l = (a-b).length();
        if ((a-c).length() <= l && (b-c).length() <= l)
        {
            return true;
        }
        return false;
    }
    
    /**
     Returns whether the point p is between the points in the vector corners.
     */
    inline bool is_inside(CGLA::Vec2d p, std::vector<CGLA::Vec2d> corners)
    {
        CGLA::Vec2d c0, c1, c2;
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
            } while (Util::is_left_of(c0,c1,c2));
            
            if(!Util::is_left_of(c0, c1, p) && !Util::is_left_of(c1, c2, p) && !Util::is_left_of(c2, c0, p))
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
    inline double intersection(CGLA::Vec2d p, CGLA::Vec2d r, CGLA::Vec2d q, CGLA::Vec2d s)
    {
        double t = INFINITY;
        double a = cross(q-p, s);
        double b = cross(r,s);
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
            double u = cross(q-p, r)/b;
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
    inline CGLA::Vec3d jet_color(double value)
    {
        double fourValue = 4 * value;
        double red   = std::min(fourValue - 1.5, -fourValue + 4.5);
        double green = std::min(fourValue - 0.5, -fourValue + 3.5);
        double blue  = std::min(fourValue + 0.5, -fourValue + 2.5);
        CGLA::Vec3d col(red, green, blue);
        for (int i = 0; i < 3; i++)
        {
            col[i] = std::max(std::min(col[i] , 1.), 0.);
        }
        return col;
    }
    
    inline CGLA::Vec3d color(CGLA::Vec3d start_color, int i)
    {
        if (i <= 0) {
            return start_color;
        }
        i++;
        double step = static_cast<double>(i - i%3)/3.;
        CGLA::Vec3d col = start_color;
        col[i%3] += 0.7*step;
        col[i%3] -= std::floor(col[i%3]);
        return col;
    }
    
    
    /**
     Returns the sign of the input val.
     */
    template <typename T>
    inline int sgn(T val) {
        return (T(0) < val) - (val < T(0));
    }
    
    
    /**
     Returns the maximum difference between the values x[i] and y[i] for all i less than the size of x and y.
     */
    inline double max_diff(const std::vector<double>& x, const std::vector<double>& y)
    {
        double temp, diff = -INFINITY;
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

#endif
