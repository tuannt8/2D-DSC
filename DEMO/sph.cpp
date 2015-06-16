//
//  sph.cpp
//  DSC
//
//  Created by Tuan Nguyen Trung on 6/16/15.
//  Copyright (c) 2015 Asger Nyman Christiansen. All rights reserved.
//

#include "sph.h"
#include "object_generator.h"

using DSC2D::vec2;

void draw_point_region(DSC2D::vec2 pos, double r){
    
    DSC2D::vec2 pts[4] = {pos + vec2(-r,-r)
                        , pos + vec2(r,-r)
                        , pos + vec2(r,r)
                        , pos + vec2(-r, r)};
    // Interial
    glBegin(GL_QUADS);
    for (int i = 0; i<4; i++) {
        glVertex2d(pts[i][0], pts[i][1]);
    }
    glEnd();
    // Edges
    glBegin(GL_LINES);
    for (int i = 0; i < 4; i++) {
        glVertex2d(pts[i][0], pts[i][1]);
        int idx = (i+1)%4;
        glVertex2d(pts[idx][0], pts[idx][1]);
    }
    glEnd();
}

void sph::draw(){
    glPointSize(2.0);
    glColor3f(1, 0, 0);
    glBegin(GL_POINTS);
    for (auto p : point_){
        glVertex2d(p[0], p[1]);
    }
    glEnd();
}

void sph::gravity_down(){
    for (int i = 0; i < point_.size(); i++) {
        vec2 new_pos = point_[i] + GRAVITY;
        
        // Project it back to boundary
        if (new_pos[0] < ld_[0]) {
            new_pos[0] = ld_[0];
        }
        if (new_pos[0] > ru_[0]) {
            new_pos[0] = ru_[0];
        }
        if (new_pos[1] < ld_[1]) {
            new_pos[1] = ld_[1];
        }
        
        point_[i] = new_pos;
    }
}

void sph::init(DSC2D::DeformableSimplicialComplex &complex){
    // Init inside the DSC, half size
    auto corners = complex.get_design_domain()->get_corners();
    auto center = complex.get_center();
    vec2 diag = corners[2] - corners[0];
    diag = diag/3;
    
    ld_ = center - diag;
    ru_ = center + diag;
    
    // Init original SPH
    size_ = 10;
    double dx = (ru_[0] - ld_[0])/(size_-1);
    double dy = (ru_[1] - ld_[1])/(size_ - 1);
    for (int i = 0; i<size_; i++) {
        for (int j = 0; j < size_; j++) {
            vec2 pt = ld_ + vec2(i*dx, j*dy);
            point_.push_back(pt);
        }
    }
    
    radius_ = std::sqrt( (ru_[0] - ld_[0]) * (ru_[1] - ld_[1]) / size_ * size_);
    
    
    DSC2D::ObjectGenerator::create_square(complex, ld_, ru_-ld_, 1);
    
}