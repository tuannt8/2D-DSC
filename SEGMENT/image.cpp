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

#define NOISE 20.0
#define BLUR 5.0

void image::load_image(std::string const file_path){
    load(file_path.c_str());
    blur(BLUR);
    noise(NOISE);
}

// Draw in OpenGL coordinate
void image::draw_image(int window_width){
    double pointSize = (double)window_width / this->width();
    glPointSize(pointSize);
    glBegin(GL_POINTS);
    for (int i = 0; i < width(); i++) {
        for (int j = 0; j < height(); j++) {
            double g = 1.0 - get_intensity_d(i, j);
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
    return MAX_BYTE - (*this)(x, height() - y);
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

Vec2 image::get_local_norm(dsc_obj &complex, Node_key key, bool outside){
    
    Vec2 norm(0.0);
    int phase = outside? 0 : 1;
    
    if (debug_num[0] == key.get_index()) {
        
    }
    
    for (auto hew = complex.walker(key); !hew.full_circle(); hew = hew.circulate_vertex_cw()) {
        auto fid = hew.face();
        
        if (complex.get_label(fid) == phase) { // Currently take only outward pointer
            auto vids = complex.get_verts(fid);
            
            // Get bisector vector
            Vec2 bisector(0.0);
            Vec2 e[2];int num = 0;
            for (auto vid : vids) {
                if (vid != key) {
                    Vec2 n = complex.get_pos(vid) - complex.get_pos(key);
                    n.normalize();
                    bisector += n;
                    e[num++] = n;
                }
            }
            bisector.normalize();
            double cos_angle = std::sin( std::acos( DSC2D::Util::dot(e[0], e[1]) ) );
            
            auto pts = complex.get_pos(fid);
            int pixel_count = 0;
            int sum_intensity = get_triangle_intensity_count(pts, &pixel_count);
            
            double intsity = ( outside? (double)sum_intensity :
                                        (double)(pixel_count*MAX_BYTE - sum_intensity)
                              ) / (double)pixel_count;
            
//            intsity = 255 - intsity; //Black - white exchange
            
            norm += bisector * intsity * cos_angle;
        }
    }
    
    if (norm.length() > 0) {
        norm.normalize();
    }
    
    norm = norm * 0.7 + complex.get_normal(key)*(outside? 1:-1);
    norm.normalize();
    return norm;
}