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

#include "boundary_conditions.h"

void DesignDomain::clamp_position(CGLA::Vec2d& p) const
{
    if(!is_inside(p))
    {
        CGLA::Vec2d c0, c1, b, p_proj, p_int1, p_int2;
        double length, dist1 = INFINITY, dist2 = INFINITY;
        for (int i = 0; i < corners.size(); i++)
        {
            c0 = corners[i];
            c1 = corners[(i+1)%corners.size()];
            b = CGLA::normalize(c1 - c0);
            p_proj = c0 + CGLA::dot(p - c0, b)*b;
            length = CGLA::sqr_length(p - p_proj);
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

void DesignDomain::clamp_vector(const CGLA::Vec2d& p, CGLA::Vec2d& v) const
{
    if(!is_inside(p+v))
    {
        CGLA::Vec2d c0, c1;
        double t;
        for (int i = 0; i < corners.size(); i++)
        {
            c0 = corners[i];
            c1 = corners[(i+1)%corners.size()];
            t = Util::intersection(p, v, c0, c1 - c0);
            if(t >= 0. && t < 1.)
            {
                v = t*v;
            }
        }
    }
}

bool DesignDomain::is_inside(CGLA::Vec2d p) const
{
    return Util::is_inside(p, corners);
}

std::vector<CGLA::Vec2d> DesignDomain::get_corners() const
{
    return corners;
}

CGLA::Vec2d DesignDomain::get_center()
{
    CGLA::Vec2d center(0.);
    for (int i = 0; i < corners.size(); i++)
    {
        center += corners[i];
    }
    return center/static_cast<double>(corners.size());
}

double DesignDomain::get_volume()
{
    if(volume < 0.)
    {
        volume = 0.;
        std::vector<CGLA::Vec2d> c(corners);
        CGLA::Vec2d c0, c1, c2;
        while(c.size() > 2)
        {
            int i = 0;
            do {
#ifdef DEBUG
                assert(i < c.size());
#endif
                c0 = c[i];
                c1 = c[(i+1)%c.size()];
                c2 = c[(i+2)%c.size()];
                i++;
            } while (Util::is_left_of(c0,c1,c2));
            
            volume += std::abs(Util::signed_area(c0, c1, c2));
            c.erase(c.begin() + (i%c.size()));
        }
    }
    return volume;
}
