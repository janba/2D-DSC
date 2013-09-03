//
//  log.cpp
//  2D_DSC
//
//  Created by Asger Nyman Christiansen on 9/10/12.
//  Copyright (c) 2012 DTU Informatics. All rights reserved.
//

#include "log.h"


#ifdef WIN32
#include <direct.h>
#else
#include <sys/stat.h>
#include <sys/types.h>
#endif

Log::Log(std::string path_)
{
    std::string temp;
    int error;
    int i = 0;
    do {
        temp = Util::concat4digits(path_ + "_test",i);
#ifdef WIN32
		error = _mkdir(temp.c_str());
#else
        error = mkdir(temp.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
#endif
        i++;
    } while (error != 0 && i < 1000);
    path = temp;
    log.open(path + "/log.txt");
}

void Log::write_message(const char* message)
{
    log << std::endl  << "*** " << message << " ***" << std::endl;
    std::cout << "*** " << message << " ***" << std::endl;
}

void Log::write_timestep(const VelocityFunc *vel_fun)
{
//    std::cout << "\n\n*** Time step #" << vel_fun->get_time_step() << " ***" << std::endl;
    log << std::endl << "*** Time step #" << vel_fun->get_time_step() << " ***" << std::endl;
    log << std::endl;
    write_variable("Compute time", vel_fun->get_compute_time(), "s");
    write_variable("Deform time", vel_fun->get_deform_time(), "s");
    write_variable("Total time", vel_fun->get_compute_time() + vel_fun->get_deform_time(), "s");
}

void Log::write_variable(const char* name, double value)
{
    log << "\t" << name << "\t:\t" << value << std::endl;
}

void Log::write_variable(const char* name, double value, double change)
{
    log << "\t" << name << "\t:\t" << value << "\t\tChange\t:\t" << change << std::endl;
}

void Log::write_variable(const char* name, double value, const char* unit)
{
    log << "\t" << name << "\t:\t" << value << " " << unit << std::endl;
}

void Log::write_variable(const char* name, const std::vector<double>* values)
{
    if(values->size() > 0)
    {
        log << "\t" << name << " = [";
        for (auto val = values->begin(); val != values->end(); val++)
        {
            log << *val;
            if(val + 1 != values->end())
            {
                log << ",\t";
            }
        }
        log << "];" << std::endl << std::endl;
    }
}

void Log::write_log(const VelocityFunc *vel_fun)
{
    write_message("VELOCITY FUNCTION INFO");
    write_variable("Velocity", vel_fun->get_velocity());
    write_variable("Accuracy", vel_fun->get_accuracy());
    
}

void Log::write_timings(const VelocityFunc *vel_fun)
{
    double deform_time = vel_fun->get_total_deform_time();
    double compute_time = vel_fun->get_total_compute_time();
    write_message("TIMINGS");
    write_variable("Total time", deform_time + compute_time, "s");
    write_variable("Compute time", compute_time, "s");
    write_variable("Deform time", deform_time , "s");
}

void Log::write_log(const DeformableSimplicialComplex& dsc)
{
    write_message("SIMPLICIAL COMPLEX INFO");
    write_variable("Size X\t", dsc.get_size_x());
    write_variable("Size Y\t", dsc.get_size_y());
    write_variable("Avg edge length", dsc.get_avg_edge_length());
    write_variable("Min deformation", dsc.get_min_deformation());
    write_variable("Total volume", dsc.get_volume());
    write_variable("#vertices", dsc.get_no_vertices());
    write_variable("#edges\t", dsc.get_no_halfedges());
    write_variable("#faces\t", dsc.get_no_faces());
}
