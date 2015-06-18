//
//  console_debug.cpp
//  DSC
//
//  Created by Tuan Nguyen Trung on 6/18/15.
//  Copyright (c) 2015 Asger Nyman Christiansen. All rights reserved.
//

#include "console_debug.h"
#include <stdio.h>
#include <stdlib.h>
#include <iostream>

console_debug* console_debug::instance = NULL;

void _check_gl_error_o(const char *file, int line)
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

#define check_gl_error_o _check_gl_error_o(__FILE__,__LINE__)

void display_o(){
    console_debug::get_instance()->display();
}

void keyboard_o(unsigned char key, int x, int y){
    console_debug::get_instance()->keyboard(key, x, y);
}

void reshape_o(int width, int height){
    console_debug::get_instance()->reshape(width, height);
}

void visible_o(int v){
    console_debug::get_instance()->display();
}

void mouse_func_(int button, int state, int x, int y){
    console_debug::get_instance()->mouse_func(button, state, x, y);
}

void console_debug::keyboard(unsigned char key, int x, int y){
    
}

void console_debug::mouse_func(int button, int state, int x, int y){
    if (state == GLUT_UP) {
        int xidx = (int)(x/box_x);
        int yIdx = (int)((WIN_SIZE_Y - y)/box_y);
        
        int height = WIN_SIZE_Y/box_y;
        cur_over = xidx*height + yIdx;
        if (cur_over < options.size()) {
            auto it = options.begin();
            for (int i = 0 ; i <cur_over; i++) {
                it++;
            }
            it->second = !it->second;
        }
    }
}

void console_debug::display(){
    if (glutGet(GLUT_WINDOW_WIDTH) != WIN_SIZE_X || glutGet(GLUT_WINDOW_HEIGHT) != WIN_SIZE_Y) {
        return;
    }
    glClearColor(0,0,0,0);
    glClear(GL_COLOR_BUFFER_BIT);
    
    auto it = options.begin();
    for (int i = 0; i<options.size(); i++) {
        draw_box(i, it->first, it->second);
        it++;
    }
    
    glFinish();
    glutSwapBuffers();
    
    check_gl_error_o;
}


void console_debug::reshape(int width, int height){

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluOrtho2D(0, WIN_SIZE_X, 0, WIN_SIZE_Y);
    
    glViewport(0, 0, WIN_SIZE_X, WIN_SIZE_Y);
    glutReshapeWindow(WIN_SIZE_X, WIN_SIZE_Y);
}

void console_debug::visible(int v){
    if(v==GLUT_VISIBLE)
        glutPostRedisplay();
    else
        glutIdleFunc(0);
}

void console_debug::RenderString(vec2 p, const std::string &string)
{
    glRasterPos2d(p[0], p[1]);
    int size = std::min((int)string.size(), (int)box_x - 20);
    for (int n=0; n<size; ++n) {
        glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12, string[n]);
    }
}

void console_debug::draw_box(int idx, std::string name, bool state){
    
    int height = WIN_SIZE_Y/box_y;
    int y_idx = idx%height;
    int x_idx = idx/height;
    
    vec2 ld(x_idx*box_x, y_idx * box_y);
    vec2 ru(x_idx*box_x + box_x, y_idx * box_y + box_y);
    
    // Corner points
    vec2 corner[4];
    corner[0] = ld;
    corner[1] = vec2(ld[0], ru[1]);
    corner[2] = ru;
    corner[3] = vec2(ru[0], ld[1]);
    
    // Box
    if (state) {
        glColor3dv(check);
    }else{
        glColor3dv(un_check);
    }
    glBegin(GL_QUADS);
    for (int i = 0; i<4; i++) {
        glVertex2dv(corner[i].x_);
    }
    glEnd();
    
    // draw edge
    glColor3f(1, 1, 1);
    glBegin(GL_LINES);
    for (int i = 0; i< 4; i++) {
        glVertex2dv(corner[i].x_);
        glVertex2dv(corner[(i+1)%4].x_);
    }
    glEnd();
    
    // draw text
    RenderString(ld + vec2(10,4), name.c_str());
}

void console_debug::init(){

    
        glutInit(NULL, NULL);
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

        glutDisplayFunc(display_o);
        glutKeyboardFunc(keyboard_o);
        glutVisibilityFunc(visible_o);
        glutReshapeFunc(reshape_o);
        glutMouseFunc(mouse_func_);

        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();

        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();

        check_gl_error_o;
}