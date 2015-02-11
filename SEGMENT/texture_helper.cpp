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
    assert(tex != 0);
    texture.push_back(tex);
    
    int width, height;
    unsigned char* image = SOIL_load_image(filePath.c_str(),
                                        &width, &height, 0, SOIL_LOAD_RGB);
    SOIL_free_image_data(image);
    tex_sizes.push_back(Vec2(width, height));
    return tex;
}

texture_helper::texture_helper(){
    LoadTexture("test.bmp");
}

texture_helper::~texture_helper(){
    // glDeleteTexture
}

void texture_helper::map_texture(int tex_id){
    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, texture[tex_id]);
}

void texture_helper::end(){
    glDisable(GL_TEXTURE_2D);
}
