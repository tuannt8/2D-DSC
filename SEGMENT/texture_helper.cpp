//
//  texture_helper.cpp
//  DSC
//
//  Created by Tuan Nguyen Trung on 2/11/15.
//  Copyright (c) 2015 Asger Nyman Christiansen. All rights reserved.
//

#include "texture_helper.h"
#include <stdlib.h>
#include <string.h>
#include "helper.h"

#ifdef WIN32
#include <GL/glew.h>
#include <GL/glut.h>
#include <GLGraphics/SOIL.h>
#else
#include <GEL/GL/glew.h>
#include <GLUT/glut.h>
#include <GEL/GLGraphics/SOIL.h>
#endif

GLuint texture_helper::LoadTexture( const char * filename ){
    std::string filePath = std::string(DATA_PATH) + std::string(filename);

    GLuint tex = SOIL_load_OGL_texture(filePath.c_str(),
                                       SOIL_LOAD_AUTO, SOIL_CREATE_NEW_ID, SOIL_FLAG_MIPMAPS);

    
    int width, height, chanel;
    unsigned char* _image = SOIL_load_image(filePath.c_str(),
                                        &width, &height, &chanel, SOIL_LOAD_RGB);
    size_ = Vec2(width, height);
    
    // Gray image
    gray_image_ = new double[width*height];
//    for (int i = 0; i < height; i++) {
//        for (int j = 0; j < width; j++) {
//            int idx = i*width + j;
//            gray_image_[idx] = 0.0;
//            gray_image_[idx] += _image[idx*chanel];
//            gray_image_[idx] += _image[idx*chanel + 1];
//            gray_image_[idx] += _image[idx*chanel + 2];
//        }
//    }
    
    assert(tex != 0);
    texture.push_back(tex);
    tex_sizes.push_back(Vec2(width, height));
    SOIL_free_image_data(_image);
    
    image_.load(filePath.c_str());
   // image_.mirror('y');
    
    image_ = (image_.get_channel(0) + image_.get_channel(1) + image_.get_channel(2)) / 3;
    
    return tex;
}

texture_helper::texture_helper(){
    LoadTexture("test.bmp");
}

texture_helper::~texture_helper(){
    // glDeleteTexture
    if (gray_image_) {
        delete gray_image_;
    }
    
}

void texture_helper::map_texture(int tex_id){
    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, texture[tex_id]);
}

void texture_helper::end(){
    glDisable(GL_TEXTURE_2D);
}

#pragma mark - Public

void texture_helper::drawImage(){
    glBegin(GL_POINTS);
    for (int i = 0; i < image_.width(); i++) {
        for (int j = 0; j < image_.height(); j++) {
            double g =grayd(i, j);
            glColor3f(g, g, g);
            glVertex2d((double)i, (double)j);
        }
    }
    glEnd();
}

bool texture_helper::is_tri_intersect_phase(std::vector<Vec2> pts){
    Vec2 min(INFINITY, INFINITY), max(-INFINITY, -INFINITY);
    for (auto p: pts){
        min[0] = std::min(min[0], p[0]);
        min[1] = std::min(min[1], p[1]);
        max[0] = std::max(max[0], p[0]);
        max[1] = std::max(max[1], p[1]);
    }
    
    //Convert to int
    for (int i = floor(min[0]); i < ceil(max[0]); i++) {
        for (int j = floor(min[1]); j < ceil(max[1]); j++) {
            if (helper_t::is_point_in_tri(Vec2(i,j), pts)) {
                if(phase(i, j) == 1)
                    return  true;
            }
        }
    }
    
    return  false;
}

double texture_helper:: average_intensity(std::vector<Vec2> pts){
    Vec2 min(INFINITY, INFINITY), max(-INFINITY, -INFINITY);
    for (auto p: pts){
        min[0] = std::min(min[0], p[0]);
        min[1] = std::min(min[1], p[1]);
        max[0] = std::max(max[0], p[0]);
        max[1] = std::max(max[1], p[1]);
    }
    
    //Convert to int
    int num = 0;
    double count = 0;
    for (int i = floor(min[0]); i < ceil(max[0]); i++) {
        for (int j = floor(min[1]); j < ceil(max[1]); j++) {
            if (helper_t::is_point_in_tri(Vec2(i,j), pts)) {
                num ++;
                count += phase(i, j);
            }
        }
    }
    
    return count / (double)num;
}

double texture_helper::grayd(int x, int y){
    return gray(x, y) / 255.0;
}

unsigned int texture_helper::gray(int x, int y){
    int r = image_.height() - y;
    int c = x;
    return image_.data()[r * image_.width() + c];
}

int texture_helper::phase(int x, int y){
    if (gray(x, y) > 250) {
        return 0;
    }else{
        return 1;
    }
}
