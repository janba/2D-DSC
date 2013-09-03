//
//  velocity_function.h
//  2D_DSC
//
//  Created by Asger Nyman Christiansen on 9/27/12.
//  Copyright (c) 2012 DTU Informatics. All rights reserved.
//

#ifndef _DDSC_velocity_func_h
#define _DDSC_velocity_func_h

#include "DSC.h"
#include "ctime"

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
    void update_compute_time(const clock_t& compute_time_)
    {
        double t = (double)(compute_time_) / ((double)CLOCKS_PER_SEC);
        compute_time += t;
        total_compute_time += t;
    }
    /**
     Updates the time it took to deform the interface.
     */
    void update_deform_time(const clock_t& deform_time_)
    {
        double t = (double)(deform_time_) / ((double)CLOCKS_PER_SEC);
        deform_time += t;
        total_deform_time += t;
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

#endif
