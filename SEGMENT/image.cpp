//
//  image.cpp
//  DSC
//
//  Created by Tuan Nguyen Trung on 2/18/15.
//  Copyright (c) 2015 Asger Nyman Christiansen. All rights reserved.
//

#include "image.h"
#include <stdlib.h>
#include <string.h>
#include "helper.h"
#include <math.h>

#ifdef WIN32
#include <GL/glew.h>
#include <GL/glut.h>
#include <GLGraphics/SOIL.h>
#else
#include <GEL/GL/glew.h>
#include <GLUT/glut.h>
#include <GEL/GLGraphics/SOIL.h>
#endif


#define NOISE 0.0
#define BLUR 2.0

void image::load_image(std::string const file_path){
    
    
    load(file_path.c_str());
    
    // convert to 0-1 scale
   // *this = this->get_RGBtoYCbCr().channel(0);
    
//    blur(BLUR);
//    noise(NOISE);
    
    compute_gradient();
}

// Draw in OpenGL coordinate
void image::draw_image(int window_width){
    double pointSize = (double)window_width / this->width();
    glPointSize(pointSize);
    glBegin(GL_POINTS);
    for (int i = 0; i < width(); i++) {
        for (int j = 0; j < height(); j++) {
            double g = get_intensity(i, j);
            glColor3f(g, g, g);
            glVertex2d((double)i, (double)j);
        }
    }
    glEnd();
    glPointSize(1.0);
}

void image::draw_grad(int window_width){
    double pointSize = (double)window_width / this->width();
    glPointSize(pointSize);
    glBegin(GL_LINES);
    for (int i = 0; i < width(); i++) {
        for (int j = 0; j < height(); j++) {
            Vec2 g = grad(i, j)*10;
            glColor3f(1.0, 0.0, 0.0);
            
            Vec2 pt((double)i+0.5, (double)j+0.5);
            glVertex2d(pt[0], pt[1]);
            Vec2 top = pt + g;
            glVertex2d(top[0], top[1]);
        }
    }
    glEnd();
    glPointSize(1.0);
}

// 0 - 1.0
double image::get_intensity(int x, int y){
    if (x < 0 or x > width()
        or y < 0 or y > height()) {
        return 0;
    }
    return ((double)(*this)(x, height() - y)) / (double)MAX_BYTE;
}

void image::get_tri_intensity(Vec2_array tris, int * total_pixel, double * total_intensity){
    Vec2 min(INFINITY, INFINITY), max(-INFINITY, -INFINITY);
    for (auto p: tris){
        min[0] = std::min(min[0], p[0]);
        min[1] = std::min(min[1], p[1]);
        max[0] = std::max(max[0], p[0]);
        max[1] = std::max(max[1], p[1]);
    }
    
    int t_pixel = 0;
    double total_inten = 0.0;
    
    for (int i = floor(min[0]); i < ceil(max[0]); i++) {
        for (int j = floor(min[1]); j < ceil(max[1]); j++) {
            if (helper_t::is_point_in_tri(Vec2(i,j), tris)) {
                t_pixel ++;
                total_inten += get_intensity(i, j);
            }
        }
    }
    
    *total_intensity = total_inten;
    *total_pixel = t_pixel;
}


void image::compute_gradient(){
    gradient_.resize(width()*height());
    
    cimg_library::CImgList<int> grad_img = get_gradient("xy", 0);
    CImg<int> *gX = grad_img.data(0);
    CImg<int> *gY = grad_img.data(1);
    
    for (int x = 0; x < width(); x++) {
        for (int y = 0; y< height(); y++) {
            double x_g = (*gX)(x, height()-y)/(double)MAX_BYTE;
            double y_g = -(*gY)(x, height()-y)/(double)MAX_BYTE;
            gradient_[y * width() + x] = Vec2(x_g,y_g);
        }
    }
  
    FILE *f = fopen("../grad.txt", "w");
    for (int x = 0; x < width(); x++) {
        for (int y = 0; y < height(); y++) {
            fprintf(f, "%f ", grad(x, y)[0]);
        }
        fprintf(f, "\n");
    }
    fclose(f);
//    gX->display("gX");
//    gY->display("gY");
//    grad_img.display();
}


Vec2 image::grad(int x, int y){
    return gradient_[y * width() + x];
}