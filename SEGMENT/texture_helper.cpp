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
    _size = Vec2(width, height);
    
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
    
    image_ = new cimg_library::CImg<unsigned int>(filePath.c_str());
    image_->mirror('y');
    
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
    
    if (image_) {
        delete image_;
    }
}

void texture_helper::map_texture(int tex_id){
    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, texture[tex_id]);
}

void texture_helper::end(){
    glDisable(GL_TEXTURE_2D);
}

void texture_helper::drawImage(){
    glBegin(GL_POINTS);
    unsigned int* data = (unsigned int*)image_->data();
    int size = image_->width() * image_->height();
    for (int i = 0; i < image_->height(); i++) {
        for (int j = 0; j < image_->width(); j++) {
            int idx = i * image_->width() + j;
        
            glColor3f(data[idx]/255.0, data[size + idx]/255.0, data[2*size + idx]/255.0);
            glVertex2d((double)j, (double)i);
        }
    }
    glEnd();
}