//
//  sample.cpp
//  2D_DSC
//
//  Created by Asger Nyman Christiansen on 2/14/13.
//  Copyright (c) 2013 Asger Nyman Christiansen. All rights reserved.
//

#include "user_interface.h"


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


int main(int argc, char** argv)
{
    UI ui(argc, argv);
    glutMainLoop();
    return 0;
}