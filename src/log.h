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
#include "DSC.h"
#include "velocity_function.h"

#include <fstream>
#include <string>

/**
 A class for logging information.
 */
class Log {
    
    std::string path;
protected:
    std::ofstream log;
    
public:
    /**
     Creates a log at the path given as input.
     */
    Log(std::string path);
    
    ~Log()
    {        
        log.close();
    }
    
    /**
     Returns the path where the log is saved.
     */
    std::string get_path()
    {
        return path;
    }
    
protected:
    /**
     Write a variable with name and value to the log.
     */
    void write_variable(const char* name, double value);
    
    /**
     Write a variable with name, value and change in value to the log.
     */
    void write_variable(const char* name, double value, double change);
    
    /**
     Write a variable with name, value and unit of the value to the log.
     */
    void write_variable(const char* name, double value, const char* unit);
    
    /**
     Write a variable with name and values to the log.
     */
    void write_variable(const char* name, const std::vector<double>* values);
    
public:
    
    /**
     Write the time step number, timings and additional time step information to the log.
     */
    virtual void write_timestep(const VelocityFunc *vel_fun);
    
    /**
     Write a message to the terminal and the log.
     */
    virtual void write_message(const char* message);
    
    /**
     Writes simplicial complex information to the log.
     */
    virtual void write_log(const DeformableSimplicialComplex& complex);
    
    /**
     Writes velocity function information to the log.
     */
    virtual void write_log(const VelocityFunc *vel_fun);
    
    /**
     Writes timings to the log.
     */
    virtual void write_timings(const VelocityFunc *vel_fun);
};
