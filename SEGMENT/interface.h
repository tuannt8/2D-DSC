//
//  interface.h
//  DSC
//
//  Created by Tuan Nguyen Trung on 2/9/15.
//  Copyright (c) 2015 Asger Nyman Christiansen. All rights reserved.
//

#pragma once

#include "velocity_function.h"
#include "DSC.h"
#include "log.h"
#include "texture_helper.h"

#ifdef WIN32
#include <GL/glew.h>
#include <GL/glut.h>
#include <Util/ArgExtracter.h>
#else
#include <GEL/Util/ArgExtracter.h>
#include <GEL/GL/glew.h>
#include <GLUT/glut.h>
#endif

class interface{
#pragma mark - Local variable
    std::unique_ptr<DSC2D::DeformableSimplicialComplex> dsc;

    
    // Windows size
    int WIN_SIZE_X;
    int WIN_SIZE_Y;
    double SCALE;
    Vec2 imageSize;
    
    // triangle size
    double DISCRETIZATION;
    
    static interface *instance;
    
    std::unique_ptr<texture_helper> tex;
    
#pragma mark - Glut display
public:
    void display();
    
    void animate();
    
    void reshape(int width, int height);
    
    void visible(int v);

    void keyboard(unsigned char key, int x, int y);
    
    void initGL();
    
#pragma mark - class functions
public:
    interface(int &argc, char** argv);
    virtual ~interface(){};
    
    static interface* get_instance(){
        return instance;
    }
  
private:
    void draw();
    void draw_image();
    void draw_coord();
    void update_title();
    
#pragma mark - data
private:
    void init_dsc();
    void init_boundary();
};