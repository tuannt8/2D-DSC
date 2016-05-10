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

#include <GL/glew.h>
#include "define.h"
#include <armadillo>

#ifdef Success
#undef Success
#endif

#include "Eigen/SparseCore"

/*
 Smooth image
 Access intensity by double-type coordinates
 */

class smooth_image
{
    typedef cimg_library::CImg<double> CImg_class;
    
    friend class texture_segment;
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
    
    // Integration
    template <typename T>
    T get_sum_on_tri_template(Vec2_array tris, std::function<T(Vec2)> get_v);
    double sum_over_tri(std::vector<Vec2> tris);
    double get_variation_tri(std::vector<Vec2> tris, double c_mean);
    double ci_temp;
public:
    // Load image from file
    void load_image(std::string file_path);
    void from_buffer(double *buf, int witdh, int height);
    
    // Bind GL texture for rendering
    void draw_image();
    void bind_texture(){glBindTexture(GL_TEXTURE_2D, _gl_texture_ID);}
    void ex_display(std::string name = "Image"){
        main_disp = cimg_library::CImgDisplay(_core_img, name.c_str());
    };
    void close_display(){main_disp.close();}
public:
    // For probability image
    smooth_image(int width, int height);
    void normalize(){_core_img.normalize(0.0,1.0);}
    void fill(double val){ _core_img.fill(val); }
    void set_value(int x, int y, double v);
    void averaging(const std::vector<std::shared_ptr<smooth_image>> imgs);
    
    Eigen::VectorXd reshape_to_vector();
    void update(Eigen::VectorXd prob);
    
    static void blur(std::vector<std::shared_ptr<smooth_image>> imgs);
    static void area_normalization(std::vector<std::shared_ptr<smooth_image>> imgs, std::vector<double> area);
    static void display_list(std::vector<std::shared_ptr<smooth_image>> imgs);
private:
    // For OpenGL texture mapping
    GLuint _gl_texture_ID = 0;
    void update_gl_texgture();
    
    cimg_library::CImgDisplay main_disp;
};

typedef std::shared_ptr<smooth_image> smooth_image_ptr;

#endif /* smooth_image_hpp */
