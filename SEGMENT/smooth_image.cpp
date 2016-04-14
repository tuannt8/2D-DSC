//
//  smooth_image.cpp
//  DSC_texture_seg
//
//  Created by Tuan Nguyen Trung on 4/11/16.
//  Copyright © 2016 Asger Nyman Christiansen. All rights reserved.
//

#include "smooth_image.hpp"

smooth_image::smooth_image(int width, int height)
{
    _core_img = CImg_class(width, height, 1, 1);
    _core_img.fill(0.5);
}

void smooth_image::load_image(std::string file_path)
{
    _core_img.load(file_path.data());
    _core_img.normalize(0,1.0);
    
    update_gl_texgture();
}

double smooth_image::get_value_f(double x, double y)
{
//    static int height = _core_img.height();
//    double y = height - 1 - y_;
    
    
    int x_i = (int)x;
    int y_i = (int)y;
    
//    if (x_i < 0 or x_i >= _core_img.width()
//        or y_i < 0 or y_i >= _core_img.height())
//    {
//        //    std::cout << "Out of image [" << x << ", " << y << "]\n";
//        return 0;
//    }
    
    double ep_x = x - x_i;
    double ep_y = y - y_i;
    
    double vdown = get_value_i(x_i, y_i)*(1-ep_x) + get_value_i(x_i + 1, y_i)*ep_x;
    double vup = get_value_i(x_i, y_i+1)*(1-ep_x) + get_value_i(x_i + 1, y_i+1)*ep_x;
    double v = vdown * (1 - ep_y) + vup * ep_y;
    
    return v;
}

double smooth_image::get_value_i(int x, int y, int channel)
{
    if (x < 0 or x >= _core_img.width()
        or y < 0 or y >= _core_img.height())
    {
        return 0;
    }
    return _core_img(x, height() - 1 - y, channel);
}

void smooth_image::set_value(int x, int y, double v)
{
    static int height = _core_img.height();
    _core_img(x,height-1-y) = v;
}

arma::vec smooth_image::reshape_to_vector()
{
    double * a = _core_img.data(0);
    arma::vec v(a, _core_img.width()*_core_img.height());
    
    return v;
}

void smooth_image::update(arma::vec prob)
{
    std::memcpy(_core_img.data(), prob.memptr(), prob.n_elem*sizeof(double));
    // TODO: ????.
//    _core_img.normalize(0, 1);
}

void smooth_image::area_normalization
    (std::vector<std::shared_ptr<smooth_image>> imgs, std::vector<double> area)
{
    int width = imgs[0]->width();
    int height = imgs[1]->height();
    
    CImg_class sum(width, height);
    sum.fill(0.0);
    for (int i = 0; i < imgs.size(); i++)
    {
        imgs[i]->_core_img /= area[i];
        sum += imgs[i]->_core_img;
    }
    

    for (int i = 0; i < imgs.size(); i++)
    {
        imgs[i]->_core_img.div(sum);
    }
}

void smooth_image::averaging(const std::vector<std::shared_ptr<smooth_image>> imgs)
{
//    _core_img = imgs[1]->_core_img;
//    update_gl_texgture();
//    return;
    
    std::vector<double> color_maps = {1.0, 0.0, 0.5, 0.2, 0.8};
    
    _core_img.fill(0.0);
    for (int i = 0; i < imgs.size(); i++)
    {
        auto iptr = imgs[i];
        _core_img = _core_img + iptr->_core_img*color_maps[i];;
    }

    _core_img.normalize(0.0, 1.0);
    
    update_gl_texgture();
}

void smooth_image::update_gl_texgture()
{
    if(_gl_texture_ID != 0)
    {
        glDeleteTextures(1, &_gl_texture_ID);
        _gl_texture_ID = 0;
    }
    
    BYTE* texture_buf = (BYTE*)malloc( _core_img.width()* _core_img.height() * 3 * sizeof(BYTE));
    BYTE* ptr = texture_buf;
    
    for (int j = 0; j < _core_img.height(); ++j) {
        for (int i = 0; i < _core_img.width(); ++i) {
            
            if(_core_img.spectrum() == 1)
            {
                BYTE color = (BYTE)(get_value_f(i, j)*MAX_BYTE);
                
                *(ptr++) = color;
                *(ptr++) = color;
                *(ptr++) = color;
            }
            else{
                *(ptr++) = (BYTE)(get_value_i(i, j, 0)*MAX_BYTE);
                *(ptr++) = (BYTE)(get_value_i(i, j, 1)*MAX_BYTE);
                *(ptr++) = (BYTE)(get_value_i(i, j, 2)*MAX_BYTE);
            }
        }
    }
    
    glGenTextures(1, &_gl_texture_ID);
    
    // "Bind" the newly created texture : all future texture functions will modify this texture
    glBindTexture(GL_TEXTURE_2D, _gl_texture_ID);
    
    // Give the image to OpenGL
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, _core_img.width(), _core_img.height(), 0, GL_RGB, GL_UNSIGNED_BYTE, texture_buf);
    
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