//
//  curve.cpp
//  DSC
//
//  Created by Tuan Nguyen Trung on 2/12/15.
//  Copyright (c) 2015 Asger Nyman Christiansen. All rights reserved.
//

#include "curve.h"
#ifdef WIN32
#include <GL/glew.h>
#include <GL/glut.h>
#include <GLGraphics/SOIL.h>
#else
#include <GEL/GL/glew.h>
#include <GLUT/glut.h>
#include <GEL/GLGraphics/SOIL.h>
#endif

#include "helper.h"

curve::curve(){
    
}

curve::~curve(){
    
}

#define POS(i) dsc.get_pos(cycle_at(i))

void curve::draw(DSC2D::DeformableSimplicialComplex &dsc, DSC2D::vec3 color){
    
    glLineWidth(1.5f);
    glColor3dv(color.get());
    glBegin(GL_LINES);
    
    for (int i = 0; i < size(); i++) {
        glVertex2dv(dsc.get_pos(at(i)).get());
        glVertex2dv(dsc.get_pos(at( (i + 1)%size() )).get());
    }
    
    glEnd();
}

void curve::update_derivative(DSC2D::DeformableSimplicialComplex &dsc){
    
    // Second derivative
    second_derivative_.clear();
    second_derivative_.resize(size());
    for (int i = 0; i < size(); i++) {
        second_derivative_[i] = POS(i-1) - POS(i)*2 + POS(i+1);
    }
    
    // Forth derivative
    forth_derivative_.clear();
    forth_derivative_.resize(size());
    for (int i = 0; i < size(); i++) {
        forth_derivative_[i] = POS(i-2) - POS(i-1)*4 + POS(i)*6 - POS(i+1)*4 + POS(i+2);
    }
}

curve::node_key curve::cycle_at(int idx){

    return (*this)[idx % size()];
}

void curve::update_mean_intensity(dsc_obj &complex, image &img){
    m_in_ = 0.0;
    m_out_ = 0.0;
    int count_in = 0, count_out = 0;
    
    for (auto fi = complex.faces_begin(); fi != complex.faces_end(); fi++) {
        auto pts = complex.get_pos(*fi);
        int count;
        int total_i = img.get_triangle_intensity_count(pts, &count);
        
        if (complex.get_label(*fi) == 0) { // Outside
            m_out_ += total_i;
            count_out += count;
        }else{
            m_in_ += total_i;
            count_in += count;
        }
    }
    
    m_in_ /= count_in;
    m_out_ /= count_out;
}