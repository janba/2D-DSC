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

#include "DSC.h"

/**
 An abstract class which a specific velocity function should inherit from.
 */
class VelocityFunc
{
    int time_step;
    int MAX_TIME_STEPS;
    
    double compute_time;
    double deform_time;
    
    double total_compute_time;
    double total_deform_time;
    
protected:
    double VELOCITY; // Determines the distance each interface vertex moves at each iteration.
    double ACCURACY; // Determines the accuracy of the final result.
    
    std::vector<CGLA::Vec2d> pos_old;
    
    /**
     Creates a velocity function which is applied to the simplicial complex. The velocity parameter determines the velocity of the function.
     */
    VelocityFunc(double velocity, double accuracy, int max_time_steps = 500):
        VELOCITY(velocity), ACCURACY(accuracy), time_step(0), MAX_TIME_STEPS(max_time_steps),
        compute_time(0.), deform_time(0.), total_compute_time(0.), total_deform_time(0.)
    {
        
    }
    
public:
    virtual ~VelocityFunc()
    {
        pos_old.clear();
    }
    
    int get_time_step() const
    {
        return time_step;
    }
    
    /**
     Returns the name of the velocity function.
     */
    virtual std::string get_name() const = 0;
    
    /**
     Returns the velocity.
     */
    double get_velocity() const
    {
        return VELOCITY;
    }
    
    /**
     Returns the accuracy.
     */
    double get_accuracy() const
    {
        return ACCURACY;
    }
    
    /**
     Returns the time it took to deform the interface in this time step.
     */
    double get_deform_time() const
    {
        return deform_time;
    }
    
    /**
     Returns the time it took to compute the new positions of the interface in this time step.
     */
    double get_compute_time() const
    {
        return compute_time;
    }
    
    /**
     Returns the total time it took to deform the interface.
     */
    double get_total_deform_time() const
    {
        return total_deform_time;
    }
    
    /**
     Returns the total time it took to compute the new positions of the interface.
     */
    double get_total_compute_time() const
    {
        return total_compute_time;
    }
    
protected:
    /**
     Updates the time it took to compute new positions for the interface vertices.
     */
    void update_compute_time(const std::chrono::time_point<std::chrono::system_clock>& start_time)
    {
        std::chrono::duration<double> t = std::chrono::system_clock::now() - start_time;
        compute_time += t.count();
        total_compute_time += t.count();
    }
    /**
     Updates the time it took to deform the interface.
     */
    void update_deform_time(const std::chrono::time_point<std::chrono::system_clock>& start_time)
    {
        std::chrono::duration<double> t = std::chrono::system_clock::now() - start_time;
        deform_time += t.count();
        total_deform_time += t.count();
    }
    
private:
    /**
     Computes the motion of each interface vertex and stores the new position in new_pos in the simplicial complex class.
     */
    virtual void deform(DeformableSimplicialComplex& complex) = 0;
    
public:
    void take_time_step(DeformableSimplicialComplex& complex)
    {
        compute_time = 0.;
        deform_time = 0.;
        deform(complex);
        time_step++;
    }
    
    /**
     Returns whether the motion has finished.
     */
    virtual bool is_motion_finished(DeformableSimplicialComplex& complex);
    
    /**
     An optional test function which can be used to test some aspect of the velocity function.
     */
    virtual void test(DeformableSimplicialComplex& complex)
    {
        
    }
    
};
