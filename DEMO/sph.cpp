//
//  sph.cpp
//  DSC
//
//  Created by Tuan Nguyen Trung on 6/16/15.
//  Copyright (c) 2015 Asger Nyman Christiansen. All rights reserved.
//

#include "sph.h"
#include "object_generator.h"
#include "console_debug.h"

using DSC2D::vec2;
using DSC2D::vec3;

// 6 points integration
double gauss_W[6] = {0.10995, 0.10995, 0.10995, 0.22338,  0.22338,  0.22338};
vec3 gauss_xi[6] = {vec3(0.81684, 0.091576, 0.091576),
                    vec3(0.091576, 0.81684, 0.091576),
                    vec3(0.091576, 0.091576, 0.81684),
                    vec3(0.1081, 0.4459, 0.4459),
                    vec3(0.4459, 0.1081, 0.4459),
                    vec3(0.4459, 0.4459, 0.1081)
                    };

void draw_point_region(DSC2D::vec2 pos, double r){
    
    /*
     Draw a square
     */
//    DSC2D::vec2 pts[4] = {pos + vec2(-r,-r)
//                        , pos + vec2(r,-r)
//                        , pos + vec2(r,r)
//                        , pos + vec2(-r, r)};
//    // Interial
//    glColor4f(1.0, 0.0, 0.0, 0.1);
//    glBegin(GL_QUADS);
//    for (int i = 0; i<4; i++) {
//        glVertex2d(pts[i][0], pts[i][1]);
//    }
//    glEnd();
    
    /*
     Draw circle
     */
    glColor4f(1.0, 0.0, 0.0, 0.1);
    float x,y;
    glBegin(GL_TRIANGLES);
    
    int num = 20;
    float dd = 360./(float)num;
    x = pos[0] + (float)r * cos(0);
    y = pos[1] + (float)r * sin(0);
    for(int j = 0; j < num; j++)
    {
        glVertex2f(x,y);
        x = pos[0] + (float)r * cos(j * dd);
        y = pos[1] + (float)r * sin(j * dd);
        glVertex2f(pos[0], pos[1]);
        glVertex2f(x,y);
    }
    glEnd();
}

double sph::omega(double r){
//    { // Cubic
//        double xi = r / (double)radius_;
//        double w;
//        if (xi >= 0 or xi <= 1) {
//            w = 1 - 3.0/2.0*xi*xi + 3.0/4.0*xi*xi*xi;
//        } else if (xi <= 2){
//            w = std::pow( 2-xi , 3) / 4.0;
//        } else
//            w = 0.0;
//        
//        return w/(radius_*radius_*radius_*3.14159);
//    }
    { // Gaussian
        if (r < radius_) {
            return 15/(3.14 * std::pow(radius_, 6))*std::pow((radius_-r), 3);
        }
        else
            return 0;
    }
}


double sph::get_mass_tri(std::vector<DSC2D::vec2> pts){
    // Use 6 point integrationi
    double rho = 0.0;
    for (int i = 0; i < 6; i++) {
        vec2 pt(0.0);
        for (int j = 0; j<3; j++) {
            pt = pt + pts[j]*gauss_xi[i][j];
        }
        
        rho += gauss_W[i] * get_intensity(pt);
    }
    rho /= 4.0;
    double V = std::abs(CGLA::cross(pts[1] - pts[0], pts[2] - pts[0]));
    
    return rho*V;
}

void sph::draw(){
    if (console_debug::get_opt("Particle region", false)) {
        for (auto p: point_){
            draw_point_region(p, radius_);
        }
    }

    if (console_debug::get_opt("Particle points", true)) {
        glPointSize(2.0);
        glColor3f(1, 0, 0);
        glBegin(GL_POINTS);
        for (auto p : point_){
            glVertex2d(p[0], p[1]);
        }
        glEnd();
    }
}

double sph::get_intensity(DSC2D::vec2 pt){
    // TODO: Performance gain later
    double rho = 0;
    
    for (int i = 0; i < point_.size(); i++) {
        double r = (point_[i] - pt).length();
        if (r < 2*radius_) {
            rho += omega(r) * mass;
        }
    }
    
    return rho;
}

double sph::get_total_mass(){
    return mass * point_.size();
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
    diag = diag/3.5;
    
    ld_ = center - diag;
    ru_ = center + diag;
    
    // Init original SPH
//    size_ = 30;
    double dx = (ru_[0] - ld_[0])/(size_-1);
    double dy = (ru_[1] - ld_[1])/(size_ - 1);
    for (int i = 0; i<size_; i++) {
        for (int j = 0; j < size_; j++) {
            vec2 pt = ld_ + vec2(i*dx, j*dy);
            point_.push_back(pt);
        }
    }
    
    radius_ = std::sqrt(
                        (ru_[0] - ld_[0]) * (ru_[1] - ld_[1])
                        / size_ / size_)
                * 2.0 / 1.0;
    
    DSC2D::vec2 gap(10,10);
    DSC2D::ObjectGenerator::create_square(complex, ld_ - gap, ru_- ld_ + gap, 1);
    
}