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

#include "user_interface.h"

void _check_gl_error(const char *file, int line)
{
    GLenum err (glGetError());
    
    while(err!=GL_NO_ERROR) {
        std::string error;
        
        switch(err) {
            case GL_INVALID_OPERATION:      error="INVALID_OPERATION";      break;
            case GL_INVALID_ENUM:           error="INVALID_ENUM";           break;
            case GL_INVALID_VALUE:          error="INVALID_VALUE";          break;
            case GL_OUT_OF_MEMORY:          error="OUT_OF_MEMORY";          break;
            case GL_INVALID_FRAMEBUFFER_OPERATION:  error="INVALID_FRAMEBUFFER_OPERATION";  break;
        }
        
        std::cerr << "GL_" << error.c_str() <<" - "<<file<<":"<<line<<std::endl;
        err=glGetError();
    }
}

#define check_gl_error() _check_gl_error(__FILE__,__LINE__)


void display_(){
    UI::get_instance()->display();
}

void keyboard_(unsigned char key, int x, int y){
    UI::get_instance()->keyboard(key, x, y);
}

void reshape_(int width, int height){
    UI::get_instance()->reshape(width, height);
}

void visible_(int v){
    UI::get_instance()->visible(v);
}

void animate_(){
    UI::get_instance()->animate();
}

UI* UI::instance = NULL;

UI::UI(int &argc, char** argv)
{
    instance = this;
	WIN_SIZE_X = 500;
    WIN_SIZE_Y = 500;

    glutInit(&argc, argv);
    glutInitWindowSize(WIN_SIZE_X,WIN_SIZE_Y);
#ifdef WIN32
    glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_MULTISAMPLE);
#else
    glutInitDisplayString("rgba double samples=16");
#endif
    glutCreateWindow("");
    
    glEnable(GL_MULTISAMPLE);
	
	glEnable(GL_LINE_SMOOTH);
    glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
    glEnable(GL_POINT_SMOOTH);
    glHint(GL_POINT_SMOOTH_HINT, GL_NICEST);
	glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    glutDisplayFunc(display_);
    glutKeyboardFunc(keyboard_);
    glutVisibilityFunc(visible_);
    glutReshapeFunc(reshape_);
    
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    
    dsc = nullptr;
    vel_fun = nullptr;
    
    if(argc > 1)
    {
        QUIT_ON_COMPLETION = true;
        CONTINUOUS = true;
        RECORD = true;
        
        Util::ArgExtracter ext(argc, argv);
        ext.extract("nu", VELOCITY);
        ext.extract("delta", DISCRETIZATION);
        ext.extract("alpha", ACCURACY);
    }
    else {
        VELOCITY = 10.;
        DISCRETIZATION = 25.;
        ACCURACY = 5.;
        
        CONTINUOUS = false;
        RECORD = true;
        QUIT_ON_COMPLETION = false;
    }
    update_title();
    check_gl_error();
}

void UI::update_title()
{
    std::ostringstream oss;
    oss << "2D DSC\t";
    if(vel_fun)
    {
        oss << vel_fun->get_name();
        oss << ", Time step " << vel_fun->get_time_step();
    }
    oss << " (Nu = " << VELOCITY << ", Delta = " << DISCRETIZATION << ", Alpha = " << ACCURACY << ")";
    std::string str(oss.str());
    glutSetWindowTitle(str.c_str());
}

void UI::display()
{
    if (glutGet(GLUT_WINDOW_WIDTH) != WIN_SIZE_X || glutGet(GLUT_WINDOW_HEIGHT) != WIN_SIZE_Y) {
        return;
    }
    
    draw();
    update_title();
    
    if(vel_fun && CONTINUOUS)
    {
        vel_fun->take_time_step(*dsc);
        basic_log->write_timestep(vel_fun);
        if (vel_fun->is_motion_finished(*dsc))
        {
            stop();
            if (QUIT_ON_COMPLETION) {
                exit(0);
            }
        }
    }
    check_gl_error();
}

void UI::reshape(int width, int height)
{
    if(dsc)
    {
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        gluOrtho2D(0, WIN_SIZE_X, 0, WIN_SIZE_Y);
    }
    
    glViewport(0, 0, WIN_SIZE_X, WIN_SIZE_Y);
    glutReshapeWindow(WIN_SIZE_X, WIN_SIZE_Y);
}

void UI::animate()
{
    glutPostRedisplay();
}

void UI::keyboard(unsigned char key, int x, int y) {
    switch(key) {
        case '\033':
            stop();
            exit(0);
            break;
        case '0':
            stop();
            break;
        case '1':
            rotate_square();
            break;
        case '2':
            smooth_filled();
            break;
        case '3':
            expand_blobs();
            break;
        case ' ':
            if(!CONTINUOUS)
            {
                std::cout << "MOTION STARTED" << std::endl;
            }
            else {
                std::cout << "MOTION PAUSED" << std::endl;
            }
            CONTINUOUS = !CONTINUOUS;
            break;
        case 'm':
            if(vel_fun)
            {
                std::cout << "MOVE" << std::endl;
                vel_fun->take_time_step(*dsc);
            }
            break;
        case 't':
            if(vel_fun)
            {
                std::cout << "TEST" << std::endl;
                vel_fun->test(*dsc);
            }
            break;
        case '\t':
            if(dsc)
            {
                switch_display_type();
            }
            break;
        case 's':
            if(dsc)
            {
                std::cout << "TAKING SCREEN SHOT" << std::endl;
                Painter::save_painting(WIN_SIZE_X, WIN_SIZE_Y, "LOG");
            }
            break;
        case '+':
            if(!vel_fun)
            {
                VELOCITY = std::min(VELOCITY + 1., 100.);
                update_title();
            }
            break;
        case '-':
            if(!vel_fun)
            {
                VELOCITY = std::max(VELOCITY - 1., 0.);
                update_title();
            }
            break;
        case '.':
            if(!dsc)
            {
                DISCRETIZATION = std::min(DISCRETIZATION + 0.5, 100.);
                update_title();
            }
            break;
        case ',':
            if(!dsc)
            {
                DISCRETIZATION = std::max(DISCRETIZATION - 0.5, 1.);
                update_title();
            }
            break;
        case '<':
            if(!vel_fun)
            {
                ACCURACY = std::min(ACCURACY + 1., 100.);
                update_title();
            }
            break;
        case '>':
            if(!vel_fun)
            {
                ACCURACY = std::max(ACCURACY - 1., 1.);
                update_title();
            }
            break;
    }
}

void UI::visible(int v)
{
    if(v==GLUT_VISIBLE)
        glutIdleFunc(animate_);
    else
        glutIdleFunc(0);
}

void UI::draw()
{
    Painter::begin();
    if (dsc)
    {
        Painter::draw_complex(*dsc);
        if(RECORD && CONTINUOUS)
        {
            Painter::save_painting(WIN_SIZE_X, WIN_SIZE_Y, basic_log->get_path(), vel_fun->get_time_step());
        }
    }
    Painter::end();
}

void UI::stop()
{
    if(dsc)
    {
        draw();
        basic_log->write_message("MOTION STOPPED");
        basic_log->write_log(*dsc);
        basic_log->write_log(vel_fun);
        basic_log->write_timings(vel_fun);
        delete vel_fun;
        delete dsc;
        delete basic_log;
        vel_fun = nullptr;
        dsc = nullptr;
    }
}


using DSC2D::DeformableSimplicialComplex;
using DSC2D::Trializer;
using DSC2D::DesignDomain;
using DSC2D::ObjectGenerator;
using DSC2D::Log;
using DSC2D::real;
using DSC2D::vec2;

void UI::rotate_square()
{
    stop();
    int width = 450;
    int height = 450;
    
    std::vector<real> points;
    std::vector<int> faces;
    Trializer::trialize(width, height, DISCRETIZATION, points, faces);
    
    DesignDomain *domain = new DesignDomain(DesignDomain::RECTANGLE, width, height, DISCRETIZATION);
    
    dsc = new DeformableSimplicialComplex(DISCRETIZATION, points, faces, domain);
    vel_fun = new RotateFunc(VELOCITY, ACCURACY);
    basic_log = new Log(create_log_path());
    
    ObjectGenerator::create_square(*dsc, vec2(150., 150.), vec2(200., 200.), 1);
    
    basic_log->write_message(vel_fun->get_name().c_str());
    basic_log->write_log(*dsc);
    basic_log->write_log(vel_fun);
    
    update_title();
    reshape(width + 2*DISCRETIZATION, height + 2*DISCRETIZATION);
}

void UI::smooth_filled()
{
    stop();
    int width = 450;
    int height = 450;
    
    std::vector<real> points;
    std::vector<int> faces;
    Trializer::trialize(width, height, DISCRETIZATION, points, faces);
    
    DesignDomain *domain = new DesignDomain(DesignDomain::RECTANGLE, width, height, DISCRETIZATION);
    
    dsc = new DeformableSimplicialComplex(DISCRETIZATION, points, faces, domain);
    vel_fun = new AverageFunc(VELOCITY, ACCURACY);
    basic_log = new Log(create_log_path());
    
    ObjectGenerator::create_square(*dsc, vec2(DISCRETIZATION, DISCRETIZATION), vec2(width, height), 1);
    
    basic_log->write_message(vel_fun->get_name().c_str());
    basic_log->write_log(*dsc);
    basic_log->write_log(vel_fun);
    
    update_title();
    reshape(width + 2*DISCRETIZATION, height + 2*DISCRETIZATION);
}

void UI::expand_blobs()
{
    stop();
    int width = 450;
    int height = 450;
    
    std::vector<real> points;
    std::vector<int> faces;
    
    Trializer::trialize(width, height, DISCRETIZATION, points, faces);
    
    DesignDomain *domain = new DesignDomain(DesignDomain::RECTANGLE, width, height, DISCRETIZATION);
    
    dsc = new DeformableSimplicialComplex(DISCRETIZATION, points, faces, domain);
    vel_fun = new NormalFunc(VELOCITY, ACCURACY);
    basic_log = new Log(create_log_path());
    
    ObjectGenerator::create_blob(*dsc, vec2(200., 200.), 100., 1);
    ObjectGenerator::create_blob(*dsc, vec2(300., 400.), 50., 2);
    ObjectGenerator::create_blob(*dsc, vec2(400., 100.), 30., 3);
    
    basic_log->write_message(vel_fun->get_name().c_str());
    basic_log->write_log(*dsc);
    basic_log->write_log(vel_fun);
    
    update_title();
    reshape(width + 2*DISCRETIZATION, height + 2*DISCRETIZATION);
}

