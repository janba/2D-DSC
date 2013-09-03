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

#ifndef ___D_DSC__DSC_UI__
#define ___D_DSC__DSC_UI__

#include "rotate_function.h"
#include "average_function.h"
#include "normal_function.h"

#include "velocity_function.h"
#include "DSC.h"
#include "log.h"
#include "draw.h"

#ifdef WIN32
#include <GL/glew.h>
#include <GL/glut.h>
#include <Util/ArgExtracter.h>
#else
#include <GEL/Util/ArgExtracter.h>
#include <GEL/GL/glew.h>
#include <GLUT/glut.h>
#endif

/**
 A default user interface which utilize OpenGL, GLEW and GLUT. At least some of the motion functions should be overridden.
 */
class UI
{
protected:
    VelocityFunc *vel_fun;
    DeformableSimplicialComplex *dsc;
    Log *basic_log;
    
    int WIN_SIZE_X;
    int WIN_SIZE_Y;
    
    bool CONTINUOUS;
    bool RECORD;
    bool QUIT_ON_COMPLETION;
    
    double VELOCITY;
    double DISCRETIZATION;
    double ACCURACY;
    static UI* instance;
    
public:
    
    UI(int &argc, char** argv);
    
    static UI* get_instance()
    {
        return instance;
    }
    
    virtual std::string create_log_path()
    {
        std::ostringstream s;
        s << "LOG/delta" << DISCRETIZATION << "_nu" << VELOCITY << "_alpha" << ACCURACY;
        return s.str();
    }
    
    virtual void display();
    
    virtual void animate();
    
    virtual void reshape(int width, int height);
    
    virtual void visible(int v);
    
    /**
     The keyboard is used for all inputs.
     The workflow is to select parameters (discretization, velocity and accuracy), then the type of motion (the type of velocity function) and finally start the motion.
     A complete list of options are:
     
     *** SELECT PARAMETERS ***
     ,:         Decreases discretization by 0.5 to a minimum of 1.
     .:         Increases discretization by 0.5 to a maximum of 100.
     -:         Decreases velocity by 1 to a minimum of 1.
     +:         Increases velocity by 1 to a maximum of 100.
     >:         Decreases accuracy by 1 to a minimum of 1.
     <:         Increases accuracy by 1 to a maximum of 100.
     
     *** START/STOP MOTION ***
     SPACE:     Starts/pauses the current motion.
     0:         Stops the current motion.
     ESCAPE:    Stops the current motion and exits the application
     m:         Moves the interface vertices one time step according to the current velocity function.
     
     *** MISCELLANEOUS ***
     t:         Performs a test on the current velocity function.
     s:         Takes a screen shot.
     TAB:       Switches the display type.
     
     *** SELECT MOTION ***
     1:         Selects motion type 1.
     2:         Selects motion type 2.
     3:         Selects motion type 3.
     4:         Selects motion type 4.
     5:         Selects motion type 5.
     6:         Selects motion type 6.
     */
    virtual void keyboard(unsigned char key, int x, int y);
    
    
protected:
    virtual void motion1()
    {
        stop();
        dsc = new DeformableSimplicialComplex(450, 450, DeformableSimplicialComplex::RECTANGLE, DeformableSimplicialComplex::SQUARE, DISCRETIZATION);
        vel_fun = new RotateFunc(VELOCITY, ACCURACY);
        basic_log = new Log(create_log_path());
        
        basic_log->write_message(vel_fun->get_name().c_str());
        basic_log->write_log(*dsc);
        basic_log->write_log(vel_fun);
        
        update_title();
        reshape(WIN_SIZE_X, WIN_SIZE_Y);
    }
    
    virtual void motion2()
    {
        stop();
        dsc = new DeformableSimplicialComplex(450, 450, DeformableSimplicialComplex::RECTANGLE, DeformableSimplicialComplex::FILLED, DISCRETIZATION);
        vel_fun = new AverageFunc(VELOCITY, ACCURACY);
        basic_log = new Log(create_log_path());
        
        basic_log->write_message(vel_fun->get_name().c_str());
        basic_log->write_log(*dsc);
        basic_log->write_log(vel_fun);
        
        update_title();
        reshape(WIN_SIZE_X, WIN_SIZE_Y);
    }
    
    virtual void motion3()
    {
        stop();
        dsc = new DeformableSimplicialComplex(450, 450, DeformableSimplicialComplex::RECTANGLE, DeformableSimplicialComplex::BLOBS, DISCRETIZATION);
        vel_fun = new NormalFunc(VELOCITY, ACCURACY);
        basic_log = new Log(create_log_path());
        
        basic_log->write_message(vel_fun->get_name().c_str());
        basic_log->write_log(*dsc);
        basic_log->write_log(vel_fun);
        
        update_title();
        reshape(WIN_SIZE_X, WIN_SIZE_Y);
    }
    
    /**
     Updates the window title.
     */
    virtual void update_title();
    
    /**
     Switches between different types of display if implemented.
     */
    virtual void switch_display_type()
    {
        
    }
    
    /**
     Draws the simplicial complex.
     */
    virtual void draw();
    
    /**
     Stops the motion and deletes the DSC object.
     */
    virtual void stop();
};
#endif /* defined(___D_DSC__DSC_UI__) */
