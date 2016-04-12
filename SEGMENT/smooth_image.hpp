//
//  smooth_image.hpp
//  DSC_texture_seg
//
//  Created by Tuan Nguyen Trung on 4/11/16.
//  Copyright © 2016 Asger Nyman Christiansen. All rights reserved.
//

#ifndef smooth_image_hpp
#define smooth_image_hpp

#include <stdio.h>
#include "CImg.h"
#include <GL/glew.h>
#include "define.h"

/*
 Smooth image
 Access intensity by double-type coordinates
 */

class smooth_image
{
    typedef cimg_library::CImg<double> CImg_class;
    
private: // Inheritances from CImg class
    CImg_class _core_img;
    
public:
    int width(){return _core_img.width();}
    int height(){return _core_img.height();}
    // Interpretor [0:1]
    double get_value_f(double x, double y);
    
public:
    // Load image from file
    void load_image(std::string file_path);
    
    // Bind GL texture for rendering
    void draw_image();
    void bind_texture(){glBindTexture(GL_TEXTURE_2D, _gl_texture_ID);}
private:
    // For OpenGL texture mapping
    GLuint _gl_texture_ID;
    void update_gl_texgture();
};

#endif /* smooth_image_hpp */
