//
//  smooth_image.cpp
//  DSC_texture_seg
//
//  Created by Tuan Nguyen Trung on 4/11/16.
//  Copyright © 2016 Asger Nyman Christiansen. All rights reserved.
//

#include "smooth_image.hpp"

void smooth_image::load_image(std::string file_path)
{
    _core_img.load(file_path.data());
    _core_img.normalize(0,1.0);
    
    update_gl_texgture();
}

double smooth_image::get_value_f(double x, double y)
{
    if (x < 0 or x > _core_img.width()
        or y < 0 or y > _core_img.height())
    {
        //    std::cout << "Out of image [" << x << ", " << y << "]\n";
        return 0;
    }
    
    int x_i = (int)x;
    int y_i = (int)y;
    double ep_x = x - x_i;
    double ep_y = y - y_i;
    
    double vdown = _core_img(x_i, y_i)*(1-ep_x) + _core_img(x_i + 1, y_i)*ep_x;
    double vup = _core_img(x_i, y_i+1)*(1-ep_x) + _core_img(x_i + 1, y_i+1)*ep_x;
    double v = vdown * (1 - ep_y) + vup * ep_y;
    
    return v;
}

void smooth_image::update_gl_texgture()
{
    BYTE* texture_buf = (BYTE*)malloc( _core_img.width()* _core_img.height() * 3 * sizeof(BYTE));
    BYTE* ptr = texture_buf;
    
    for (int j = 0; j < _core_img.height(); ++j) {
        for (int i = 0; i < _core_img.width(); ++i) {
            
            BYTE color = (BYTE)(_core_img(i,j)*MAX_BYTE);
            
            *(ptr++) = color;
            *(ptr++) = color;
            *(ptr++) = color;
        }
    }
    
    glGenTextures(1, &_gl_texture_ID);
    
    // "Bind" the newly created texture : all future texture functions will modify this texture
    glBindTexture(GL_TEXTURE_2D, _gl_texture_ID);
    
    // Give the image to OpenGL
    glTexImage2D(GL_TEXTURE_2D, 0,GL_RGB, _core_img.width(), _core_img.height(), 0, GL_RGB, GL_UNSIGNED_BYTE, texture_buf);
    
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
}

void smooth_image::draw_image()
{
    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, _gl_texture_ID);
    
    float w = (float)_core_img.width();
    float h = (float)_core_img.height();
    
    glColor3f(1, 1, 1);
    glBegin(GL_QUADS);
    
    glTexCoord2f(0, 0);
    glVertex2f(0, 0);
    
    glTexCoord2f(1, 0);
    glVertex2f(w, 0);
    
    glTexCoord2f(1, 1);
    glVertex2f(w, h);
    
    glTexCoord2f(0, 1);
    glVertex2f(0, h);
    
    glEnd();
    
    glDisable(GL_TEXTURE_2D);
}