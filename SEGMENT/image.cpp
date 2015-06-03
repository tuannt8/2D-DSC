//
//  image.cpp
//  DSC
//
//  Created by Tuan Nguyen Trung on 2/18/15.
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


#define NOISE 50
#define BLUR 0.0

void image::load_image(std::string const file_path){
    
    
    load(file_path.c_str());
    this->mirror('y');

//    printf("Image %d change \n", this->spectrum());

//    
//    blur(BLUR);
    noise(NOISE);

    set_gl_texture();
//    compute_gradient();
}

// Draw in OpenGL coordinate
void image::draw_image(int window_width){
    if(0){
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
    else{
        double w = width();
        double h = height();

        glEnable(GL_TEXTURE_2D);
        glBindTexture(GL_TEXTURE_2D, tex_ID);
        
        glColor3f(1, 1, 1);
        glBegin(GL_QUADS);

        glTexCoord2f(0.0, 0.0);
        glVertex2f(0.0, 0.0);

        glTexCoord2f(1.0, 0.0);
        glVertex2f(w, 0.0);

        glTexCoord2f(1.0, 1.0);
        glVertex2f(w, h);

        glTexCoord2f(0.0, 1.0);
        glVertex2f(0.0, h);
        
        glEnd();

        glDisable(GL_TEXTURE_2D);
    }

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
    return ((double)(*this)(x, y)) / (double)MAX_BYTE;
}

// 0 - 1.0
double image::get_intensity_f(double x, double y){
    if (x < 0 or x > width()
        or y < 0 or y > height()) {
        return 0;
    }
    
    int x_i = (int)x;
    int y_i = (int)y;
    double ep_x = x - x_i;
    double ep_y = y - y_i;

    double vdown = get_intensity(x_i, y_i)*(1-ep_x) + get_intensity(x_i + 1, y_i)*ep_x;
    double vup = get_intensity(x_i, y_i+1)*(1-ep_x) + get_intensity(x_i + 1, y_i+1)*ep_x;
    double v = vdown * (1 - ep_y) + vup * ep_y;
    
    return v;
}

double image::get_tri_intensity_f(Vec2_array tris, double * area_in){
    double area = helper_t::area(tris);
    int res = (int)std::sqrt(area) + 1;
    double step = 1.0/(double)res;
    
    double inten = 0.0;
    for (double ep1 = step/2.0; ep1 < 1.0; ep1 += step){
        for(double ep2 = 1 - ep1 - step/2.0; ep2 > 0; ep2 -= step){
            double ep3 = 1 - ep1 - ep2;
            Vec2 pt = tris[0]*ep1 + tris[1]*ep2 + tris[2]*ep3;
            inten += get_intensity_f(pt[0], pt[1]);
        }
        
    }
    
    if (area_in) {
        *area_in = area;
    }
    return inten;
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

intensity_out image::get_tri_differ(Vec2_array tris, double ci){
    Vec2 min(INFINITY, INFINITY), max(-INFINITY, -INFINITY);
    for (auto p: tris){
        min[0] = std::min(min[0], p[0]);
        min[1] = std::min(min[1], p[1]);
        max[0] = std::max(max[0], p[0]);
        max[1] = std::max(max[1], p[1]);
    }
    
    int t_pixel = 0;
    double total_diff = 0.0;
    
    for (int i = floor(min[0]); i < ceil(max[0]); i++) {
        for (int j = floor(min[1]); j < ceil(max[1]); j++) {
            if (helper_t::is_point_in_tri(Vec2(i,j), tris)) {
                t_pixel ++;
                total_diff += (ci - get_intensity(i, j)) * (ci - get_intensity(i, j));
            }
        }
    }
    
    intensity_out out;
    out.total_pixel = t_pixel;
    out.total_differ = total_diff;
    
    return out;
}

void image::get_tri_differ(Vec2_array tris, int *total_pixel, double * total_differ, double ci){
    Vec2 min(INFINITY, INFINITY), max(-INFINITY, -INFINITY);
    for (auto p: tris){
        min[0] = std::min(min[0], p[0]);
        min[1] = std::min(min[1], p[1]);
        max[0] = std::max(max[0], p[0]);
        max[1] = std::max(max[1], p[1]);
    }
    
    int t_pixel = 0;
    double total_diff = 0.0;
    
    for (int i = floor(min[0]); i < ceil(max[0]); i++) {
        for (int j = floor(min[1]); j < ceil(max[1]); j++) {
            if (helper_t::is_point_in_tri(Vec2(i,j), tris)) {
                t_pixel ++;
                total_diff += (ci - get_intensity(i, j)) * (ci - get_intensity(i, j));
            }
        }
    }
    
    *total_differ = total_diff;
    if(total_pixel)
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

void image::set_gl_texture() {
    BYTE* buffer = this->data();

    BYTE* texture_buf = (BYTE*)malloc( width()* height() * 3 * sizeof(BYTE));
    BYTE* ptr = texture_buf;
    for (int j = 0; j < height(); ++j) {
    for (int i = 0; i < width(); ++i) {
            *(ptr++) = (*this)(i,j, 0);
            *(ptr++) = (*this)(i,j, 0);
            *(ptr++) = (*this)(i,j, 0);
        }
    }

    glGenTextures(1, &tex_ID);

    // "Bind" the newly created texture : all future texture functions will modify this texture
    glBindTexture(GL_TEXTURE_2D, tex_ID);

    // Give the image to OpenGL
    glTexImage2D(GL_TEXTURE_2D, 0,GL_RGB, width(), height(), 0, GL_RGB, GL_UNSIGNED_BYTE, texture_buf);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
}

double image::get_tri_differ_f(Vec2_array tris, double ci) {
    double area = helper_t::area(tris);
    int res = (int)std::sqrt(area) + 1;
    double step = 1.0/(double)res;

    double energy = 0.0;
    for (double ep1 = step/2.0; ep1 < 1.0; ep1 += step){
        for(double ep2 = 1 - ep1 - step/2.0; ep2 > 0; ep2 -= step){
            double ep3 = 1 - ep1 - ep2;
            Vec2 pt = tris[0]*ep1 + tris[1]*ep2 + tris[2]*ep3;
            double g = get_intensity_f(pt[0], pt[1]);
            energy += (g - ci)*(g - ci);
        }

    }

    return energy * area / (double)(res*res);
}
