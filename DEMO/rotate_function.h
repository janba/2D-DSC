//
//  rotate_function.h
//  2D_DSC
//
//  Created by Asger Nyman Christiansen on 2/14/13.
//  Copyright (c) 2013 Asger Nyman Christiansen. All rights reserved.
//

#ifndef ___D_DSC__rotate_func__
#define ___D_DSC__rotate_func__

#include "velocity_function.h"


/**
 A rotating velocity function.
 */
class RotateFunc: public VelocityFunc {
    
public:
    /**
     Creates a rotating velocity function.
     */
    RotateFunc(double velocity, double accuracy): VelocityFunc(M_PI*velocity/180., accuracy)
    {
        
    }
    
    /**
     Returns the name of the velocity function.
     */
    virtual std::string get_name() const
    {
        return std::string("ROTATE MOTION");
    }
    
    /**
     Computes the motion of each interface vertex and stores the new position in new_pos in the simplicial complex class.
     */
    virtual void deform(DeformableSimplicialComplex& dsc);
    
    /**
     Returns wether the motion has finished.
     */
    virtual bool is_motion_finished(DeformableSimplicialComplex& dsc)
    {
        return false;
    }
    
};

#endif /* defined(___D_DSC__rotate_func__) */
