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
#include <armadillo>

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
    smooth_image(){};
    ~smooth_image(){};
    
    int width(){return _core_img.width();}
    int height(){return _core_img.height();}
    // Interpretor [0:1]
    double get_value_f(double x, double y);
    double get_value_i(int x, int y, int channel = 0);
public:
    // Load image from file
    void load_image(std::string file_path);
    
    // Bind GL texture for rendering
    void draw_image();
    void bind_texture(){glBindTexture(GL_TEXTURE_2D, _gl_texture_ID);}
    void ex_display(std::string name = "Image"){_core_img.display(name.data());};
public:
    // For probability image
    smooth_image(int width, int height);
    void fill(double val){ _core_img.fill(val); }
    void set_value(int x, int y, double v);
    void averaging(const std::vector<std::shared_ptr<smooth_image>> imgs);
    arma::vec reshape_to_vector();
    void update(arma::vec prob);
    static void area_normalization(std::vector<std::shared_ptr<smooth_image>> imgs, std::vector<double> area);
private:
    // For OpenGL texture mapping
    GLuint _gl_texture_ID = 0;
    void update_gl_texgture();
};

typedef std::shared_ptr<smooth_image> smooth_image_ptr;

#endif /* smooth_image_hpp */
