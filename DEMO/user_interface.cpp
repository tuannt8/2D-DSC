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
#include <unistd.h>

#include "rotate_function.h"
#include "average_function.h"
#include "normal_function.h"

#include "trializer.h"
#include "object_generator.h"
#include "draw.h"
#include "thread_helper.h"
#include "profile.h"

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
	WIN_SIZE_X = 800;
    WIN_SIZE_Y = 800;

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
        RECORD = false;
        
        Util::ArgExtracter ext(argc, argv);
        ext.extract("nu", VELOCITY);
        ext.extract("delta", DISCRETIZATION);
        ext.extract("alpha", ACCURACY);
        
        
    }
    else
    {
        VELOCITY = 10.;
        DISCRETIZATION = 21;
        VELOCITY = DISCRETIZATION/2.;
        ACCURACY = DISCRETIZATION/5.;
        
        CONTINUOUS = false;
        RECORD = false;
        QUIT_ON_COMPLETION = false;
        
        rotate_square();

        // Make test cases
        random_short_edge(*dsc); // good for smooth
        
//        {
//            profile pf("Parallel");
//            thread_helper th;
//            th.edge_collapse(*dsc, 0, WIN_SIZE_X);
//        }
        {

          //  dsc->remove_degenerate_edges(&count);
        //    std::cout << "Collapse " << count << " edges" << std::endl;
        }

    }
    update_title();
    check_gl_error();
}

void UI::random_short_edge(dsc_obj &complex){
    double percentage = 0.1;
    int count = 0;
    for (auto ei = complex.halfedges_begin(); ei != complex.halfedges_end(); ++ei) {
        
        double r = (double)std::rand() / (double)RAND_MAX;
        bool b_should_shorten = (r) < percentage;
        
        if (b_should_shorten && !HMesh::boundary(*complex.mesh, complex.walker(*ei).vertex())
            && !HMesh::boundary(*complex.mesh, complex.walker(*ei).opp().vertex()))
        {
            auto p1 = complex.get_pos(complex.walker(*ei).vertex());
            auto p2 = complex.get_pos(complex.walker(*ei).opp().vertex());
            double a = 0.1;
            auto p = (p1*a + p2*(1-a));
            complex.set_pos(complex.walker(*ei).vertex(), p);
            count ++;
        }
    }
    std::cout << "Shorten " << count << " edges / " << complex.get_no_halfedges() << std::endl;
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
    
    int count = 10;
    static int iter = 0;

    if(vel_fun && CONTINUOUS)
    {
        vel_fun->take_time_step(*dsc);
        basic_log->write_timestep(*vel_fun);
        if (vel_fun->is_motion_finished(*dsc))
        {
            stop();
            if (QUIT_ON_COMPLETION) {
                exit(0);
            }
        }
        iter++;
        std::cout << iter << " / " << count << std::endl;
        if (iter == count) {
            CONTINUOUS = false;
#ifdef PROFILE
            log_profile();
            std::cout << "TAKING SCREEN SHOT" << std::endl;
            Painter::save_painting(WIN_SIZE_X, WIN_SIZE_Y, "../../../LOG");
            
#endif
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

    glViewport(5, 5, WIN_SIZE_X, WIN_SIZE_Y);
    glutReshapeWindow(WIN_SIZE_X, WIN_SIZE_Y);
}

void UI::animate()
{
//    glutPostRedisplay();
}
#ifdef PROFILE
void UI::log_profile(){
    std::vector<double> time = dsc->time_profile;
    
    for(int i = 0; i < total_t; i++){
        std::cout << time[i] << " ";
    }
    
    int count = 0;
    for(auto hei = dsc->halfedges_begin(); hei != dsc->halfedges_end(); ++hei)
    {
        auto hew = dsc->walker(*hei);
        if (dsc->is_movable(hew.halfedge())
            and (dsc->is_movable(hew.vertex()) || dsc->is_movable(hew.opp().vertex())))
        {
            count++;
        }
    }
    std::cout << dsc->get_no_vertices() << " " << dsc->get_no_faces() << " " << count;
    
    const char *name_profile[] = {"Smooth"
        , "max_min_angle"
        , "remove edge"
        , "remove face"
        , "resize dsc"
        , "reset atribute"
        , "total count in while"
        , "total iterations"
    };
    
    std::cout << "# ";
    for(int i = 0; i < total_t; i++){
        std::cout << name_profile[i] << " -- ";
    }
    
    std::cout << "#vertices -- #faces -- #node on interface" << std::endl;
    
}
#endif
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
#ifdef PROFILE
                log_profile();
#endif
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
                Painter::save_painting(WIN_SIZE_X, WIN_SIZE_Y, "../../../LOG");
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
        case 'e':
            {
                {
                    profile pf("Parallel");
                    thread_helper th;
                    th.smooth(*dsc, 0, WIN_SIZE_X);
                }
                
                {
                    profile pf("serial");
                    dsc->smooth();
                }
                
                break;
            }
        case 'f':
            {
                {
                    profile pf("Parallel");
                    thread_helper th;
                    th.edge_collapse(*dsc, 0, WIN_SIZE_X);
                }
//                {
//                    profile pf("Serial");
//                    int count = 0;
//                    dsc->remove_degenerate_edges();
//                    std::cout << count << " edges collaped" << std::endl;
//                }
            }
            break;
    }
    
    glutPostRedisplay();
}

void UI::visible(int v)
{
//    if(v==GLUT_VISIBLE)
//        glutIdleFunc(animate_);
//    else
//        glutIdleFunc(0);
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
    draw();
    
    if(basic_log)
    {
        basic_log->write_message("MOTION STOPPED");
        basic_log->write_log(*dsc);
        basic_log->write_log(*vel_fun);
        basic_log->write_timings(*vel_fun);
    }
    
    basic_log = nullptr;
    vel_fun = nullptr;
    dsc = nullptr;
}

void UI::start()
{
    basic_log = std::unique_ptr<Log>(new Log(create_log_path()));
    basic_log->write_message(vel_fun->get_name().c_str());
    basic_log->write_log(*dsc);
    basic_log->write_log(*vel_fun);
    
    update_title();
}

using namespace DSC2D;

void UI::rotate_square()
{
    stop();
    int width = WIN_SIZE_X - 50;
    int height = WIN_SIZE_Y - 50;
    
    std::vector<real> points;
    std::vector<int> faces;
    Trializer::trialize(width, height, DISCRETIZATION, points, faces);
    
    DesignDomain *domain = new DesignDomain(DesignDomain::RECTANGLE, width, height, DISCRETIZATION);
    
    dsc = std::unique_ptr<DeformableSimplicialComplex>(new DeformableSimplicialComplex(DISCRETIZATION, points, faces, domain));
    vel_fun = std::unique_ptr<VelocityFunc<>>(new RotateFunc(VELOCITY, ACCURACY));
    
//    ObjectGenerator::create_square(*dsc, vec2(150., 150.), vec2(200., 200.), 1);
    
    reshape(width + 2*DISCRETIZATION, height + 2*DISCRETIZATION);
    start();
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
    
    dsc = std::unique_ptr<DeformableSimplicialComplex>(new DeformableSimplicialComplex(DISCRETIZATION, points, faces, domain));
    vel_fun = std::unique_ptr<VelocityFunc<>>(new AverageFunc(VELOCITY, ACCURACY));
    
    ObjectGenerator::create_square(*dsc, vec2(DISCRETIZATION, DISCRETIZATION), vec2(width, height), 1);
    
    reshape(width + 2*DISCRETIZATION, height + 2*DISCRETIZATION);
    start();
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
    
    dsc = std::unique_ptr<DeformableSimplicialComplex>(new DeformableSimplicialComplex(DISCRETIZATION, points, faces, domain));
    vel_fun = std::unique_ptr<VelocityFunc<>>(new NormalFunc(VELOCITY, ACCURACY));
    
    ObjectGenerator::create_blob(*dsc, vec2(200., 200.), 100., 1);
    ObjectGenerator::create_blob(*dsc, vec2(300., 400.), 50., 2);
    ObjectGenerator::create_blob(*dsc, vec2(400., 100.), 30., 3);
    
    reshape(width + 2*DISCRETIZATION, height + 2*DISCRETIZATION);
    start();
}

