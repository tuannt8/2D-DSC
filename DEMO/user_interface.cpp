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

#include "rotate_function.h"
#include "average_function.h"
#include "normal_function.h"
#include "sph_function.h"

#include "trializer.h"
#include "object_generator.h"
#include "draw.h"

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

void update_option(){
    
    glutPostWindowRedisplay(UI::get_instance()->winID);
}

#pragma mark Construction
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
    console_debug::get_instance()->init();
    console_debug::set_call_back(update_option);
    
    winID = glutCreateWindow("");
    
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
    else {
        VELOCITY = 10.;
        DISCRETIZATION = 35.;
        ACCURACY = 5.;
        
        CONTINUOUS= false;
        RECORD = false;
        QUIT_ON_COMPLETION = false;
    }
    update_title();
    check_gl_error();
    
    sph_init();


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
    draw();
    update_title();
    
    if(CONTINUOUS)
    {
       // CONTINUOUS = false;
        
        update_sph();
       
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
    
    double ratio = (double)width / (double)height;
    
    double gap = std::abs(height - width)/2;
    if (ratio <= 1) {
        glViewport(0, gap, width, height - 2*gap);
    }else{
        glViewport(gap, 0, width - 2*gap, height);
    }
    glutReshapeWindow(width, height);
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
            // rotate_square();
            sph_init();
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
        case 'd': // D ---> regular distribute
            sph_vel.deform(*dsc);
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
                save_mesh();
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
        case 'c':
            if(std::abs(DSC2D::Util::dot(sph_function::Gravity, DSC2D::vec2(1,0))) > 0.9)
           {
               sph_function::Gravity = DSC2D::vec2(0,-GRAVITY_ABS);
           }
            else{
                 sph_function::Gravity = DSC2D::vec2(-GRAVITY_ABS, 0);
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
    
    glutPostRedisplay();
}

void UI::save_mesh(){
    FILE * f = fopen("mesh.txt", "w");
    
    if (f) {
        fprintf(f, "%d %d\n", dsc->get_no_vertices(), dsc->get_no_faces());
        
        for (auto nkey : dsc->vertices()){
            auto p = dsc->get_pos(nkey);
            fprintf(f, "%f %f\n", p[0], p[1]);
        }
        
        for (auto fkey : dsc->faces()) {
            auto verts = dsc->get_verts(fkey);
            fprintf(f, "%d %d %d\n", (int)verts[0].get_index()+1,
                    (int)verts[1].get_index()+1, (int)verts[2].get_index()+1);
        }
        
        fclose(f);
    }
}

void UI::visible(int v)
{
    if(v==GLUT_VISIBLE)
        glutIdleFunc(animate_);
    else
        glutIdleFunc(0);
}

#pragma mark - DRAW

void UI::draw()
{
    Painter::begin();
    if (dsc)
    {
        
        if (console_debug::get_opt("DSC edge", true)) {
            glColor3f(0.7, 0.2, 0.4);
            Painter::draw_edges(*dsc);
        }
        
        if(console_debug::get_opt("Face index", false)){
            Painter::draw_faces_idx(*dsc);
        }
        
        sph_vel.draw();
        
     //   Painter::draw_arrows(*dsc, sph_vel.ver_dis);
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
//    basic_log = std::unique_ptr<Log>(new Log(create_log_path()));
//    basic_log->write_message(vel_fun->get_name().c_str());
//    basic_log->write_log(*dsc);
//    basic_log->write_log(*vel_fun);
//    
//    update_title();
}

using namespace DSC2D;



void UI::rotate_square()
{
    stop();
    int width = 450;
    int height = 450;
    
    std::vector<real> points;
    std::vector<int> faces;
    Trializer::trialize(width, height, DISCRETIZATION, points, faces);
    
    DesignDomain *domain = new DesignDomain(DesignDomain::RECTANGLE, width, height, DISCRETIZATION);
    
    dsc = std::unique_ptr<DeformableSimplicialComplex>
            (new DeformableSimplicialComplex(DISCRETIZATION, points, faces, domain));
    vel_fun = std::unique_ptr<VelocityFunc<>>(new RotateFunc(VELOCITY, ACCURACY));
    
    ObjectGenerator::create_square(*dsc, vec2(150., 150.), vec2(200., 200.), 1);
    
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

#pragma mark - Particle

void UI::update_sph(){
     sph_vel.deform(*dsc);
}

void UI::sph_init(){
    
    DISCRETIZATION = 20;
    
    stop();
    
    int width = WIN_SIZE_X - 2*DISCRETIZATION;
    int height = WIN_SIZE_Y - 2*DISCRETIZATION;
    
    std::vector<real> points;
    std::vector<int> faces;
    Trializer::trialize(width, height, DISCRETIZATION, points, faces);
    
    DesignDomain *domain = new DesignDomain(DesignDomain::RECTANGLE, width, height, DISCRETIZATION);
    
    dsc = std::unique_ptr<DeformableSimplicialComplex>
    (new DeformableSimplicialComplex(DISCRETIZATION, points, faces, domain));
    
    reshape(width + 2*DISCRETIZATION, height + 2*DISCRETIZATION);
    
    // sph

    sph_vel.dsc_ptr = &*dsc;
    sph_vel.init();
    
    start();
}