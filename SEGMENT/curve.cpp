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

void curve::draw(DSC2D::DeformableSimplicialComplex &dsc){
    
    glLineWidth(1.5f);
    glColor3f(1, 0, 0);
    glBegin(GL_LINES);
    
    for (int i = 0; i < size(); i++) {
        glVertex2dv(dsc.get_pos(at(i)).get());
        glVertex2dv(dsc.get_pos(at( (i + 1)%size() )).get());
    }
    
    glEnd();
}

void curve::update_derivative(DSC2D::DeformableSimplicialComplex &dsc){
    
    // Second derivative
    assert(size() > 3); //TODO: What happen if there is not enough points
    second_derivative_.clear();
    second_derivative_.resize(size());
    for (int i = 0; i < size(); i++) {
        second_derivative_[i] = POS(i-1) - POS(i)*2 + POS(i+1);
    }
    
    // Forth derivative
    assert(size() > 5); //TODO: What happen if there is not enough points
    forth_derivative_.clear();
    forth_derivative_.resize(size());
    for (int i = 0; i < size(); i++) {
        forth_derivative_[i] = POS(i-2) - POS(i-1)*4 + POS(i)*6 - POS(i+1)*4 + POS(i+2);
    }
}

curve::node_key curve::cycle_at(int idx){

    
    if(idx < 0)
        idx += size();
    
    if (idx >= size()) {
        idx -= size();
    }
    
    return at(idx);
}

void curve::update_mean_intensity(dsc_obj &complex, texture_helper &tex){
    m_in_ = 0.0;
    m_out_ = 0.0;
    int count_in = 0, count_out = 0;
    for (auto fi = complex.faces_begin(); fi != complex.faces_end(); fi++) {
        auto pts = complex.get_pos(*fi);
        int count;
        int total_i;
        tex.get_triangle_intensity(pts, count, total_i);
        
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