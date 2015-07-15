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

#include "dynamics.h"
#include "dynamics_mul.h"

class interface{
#pragma mark - Local variable
public:
    std::unique_ptr<DSC2D::DeformableSimplicialComplex> dsc;
    
    // Windows size
    int     WIN_SIZE_X;
    int     WIN_SIZE_Y;
    double  SCALE;
    Vec2    imageSize;
    std::vector<bool> bDiplay_;
    bool    RUN = false;
    
    // triangle size
    double DISCRETIZATION;
    
    static interface *instance;
    
    std::unique_ptr<dynamics_mul> dyn_;
    std::unique_ptr<image> image_;
    
private:
    int debug_num_[10];
    
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
    void thres_hold_init();
    
private:
    void draw_test();
    void draw_edge_energy();
    
    void load_dsc();
    void back_up_dsc();
    
#pragma mark - data
public:
    void init_dsc();
    void init_sqaure_boundary();
    void init_boundary();
    void dynamics_image_seg();
};