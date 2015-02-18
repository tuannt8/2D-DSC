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

void image::load_image(std::string const file_path){
    load(file_path.c_str());
}

// Draw in OpenGL coordinate
void image::draw_image(int window_width){
    double pointSize = (double)window_width / this->width();
    glPointSize(pointSize);
    glBegin(GL_POINTS);
    for (int i = 0; i < width(); i++) {
        for (int j = 0; j < height(); j++) {
            double g =get_intensity_d(i, j);
            glColor3f(g, g, g);
            glVertex2d((double)i, (double)j);
        }
    }
    glEnd();
    glPointSize(1.0);
}

// 0 - 1.0
double image::get_intensity_d(int x, int y){
    return (double)get_intensity_i(x, y) / MAX_BYTE;
}

// 0 - 255
int image::get_intensity_i(int x, int y){
    return (*this)(x, height() - y);
}

// total intensity inside a triangle
int image::get_triangle_intensity_count(Vec2_array tris, int *nb_pixel){
    Vec2 min(INFINITY, INFINITY), max(-INFINITY, -INFINITY);
    for (auto p: tris){
        min[0] = std::min(min[0], p[0]);
        min[1] = std::min(min[1], p[1]);
        max[0] = std::max(max[0], p[0]);
        max[1] = std::max(max[1], p[1]);
    }
    
    
    //Convert to int
    int total_intensity = 0;
    int total_pixel = 0;
    for (int i = floor(min[0]); i < ceil(max[0]); i++) {
        for (int j = floor(min[1]); j < ceil(max[1]); j++) {
            if (helper_t::is_point_in_tri(Vec2(i,j), tris)) {
                total_pixel ++;
                total_intensity += get_intensity_i(i, j);
            }
        }
    }
    
    if (nb_pixel) {
        *nb_pixel = total_pixel;
    }
    
    return total_intensity;
}