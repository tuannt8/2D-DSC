//
//  sph.h
//  DSC
//
//  Created by Tuan Nguyen Trung on 6/16/15.
//  Copyright (c) 2015 Asger Nyman Christiansen. All rights reserved.
//

#ifndef __DSC__sph__
#define __DSC__sph__

#include <stdio.h>
#include "DSC.h"
#include <vector>

#ifdef WIN32
#include <GL/glew.h>
#include <GL/glut.h>
#else
#include <GEL/GL/glew.h>
#include <GLUT/glut.h>
#endif


class sph{
private:
    // Dynamics points; redius and position
    double radius_;
    double mass = 0.1;
    
    // Number of sph on each size
    const int size_ = 20;
    std::vector<DSC2D::vec2> point_;
    
    // Domain
    // Left down and right up
    DSC2D::vec2 ld_, ru_;
private:
    // Constant
    const DSC2D::vec2 GRAVITY = DSC2D::vec2(0,-1);
private:
    // Check if sph go out of domain
    double omega(double r);
    
public:
    double get_intensity(DSC2D::vec2 pt);
    double get_mean_intensity_tri(std::vector<DSC2D::vec2> pts);
    double get_total_mass();
    
public:
    // draw function
    void draw();
    // Update SPH points by gravity
    void gravity_down();
    
    // Constructor
    sph(){};
    
    void init(DSC2D::DeformableSimplicialComplex &complex);
    // Destructor
    ~sph(){};
};

#endif /* defined(__DSC__sph__) */
