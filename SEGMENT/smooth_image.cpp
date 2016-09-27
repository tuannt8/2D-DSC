//
//  smooth_image.cpp
//  DSC_texture_seg
//
//  Created by Tuan Nguyen Trung on 4/11/16.
//  Copyright © 2016 Asger Nyman Christiansen. All rights reserved.
//

#include "smooth_image.hpp"
#include "helper.h"

smooth_image::smooth_image(int width, int height)
{
    _core_img = CImg_class(width, height, 1, 1);
    _core_img.fill(0.5);
}

void smooth_image::load_image(std::string file_path)
{
    try
    {
        _core_img.load(file_path.data());
        _core_img = _core_img/255.0;
        
        update_gl_texgture();
    }
    catch (std::exception e)
    {
        std::cout << e.what();
        exit(1);
    }

}

double smooth_image::get_value_f(double x, double y)
{
    
    int x_i = (int)x;
    int y_i = (int)y;
    
    double ep_x = x - x_i;
    double ep_y = y - y_i;
    
    double vdown = get_value_i(x_i, y_i)*(1-ep_x) + get_value_i(x_i + 1, y_i)*ep_x;
    double vup = get_value_i(x_i, y_i+1)*(1-ep_x) + get_value_i(x_i + 1, y_i+1)*ep_x;
    double v = vdown * (1 - ep_y) + vup * ep_y;
    
    return v;
}

template <typename T>
T smooth_image::get_sum_on_tri_template(Vec2_array tris, std::function<T(Vec2)> get_v)
{
    double area = helper_t::area(tris);
    int N = std::ceil(std::sqrt(2*area));
    double dxi = 1./(double)N;
    
    double xi1, xi2;
    Vec2 p;
    T sum = T(0.0);
    for (int i1 = 0; i1 < N; i1 ++) {
        
        xi1 = i1*dxi;
        
        for (int i2 = 0; i2 < N - i1; i2++) {
            
            xi2 = i2 * dxi;
            
            if (i1 + i2 == N - 1) {
                p = helper_t::get_coord_barry(tris, (xi1 + dxi/3.), xi2 + dxi/3.);
                sum += get_v(p)*0.5;
            }else{
                p = helper_t::get_coord_barry(tris, (xi1 + dxi/2.), xi2 + dxi/2.);
                sum += get_v(p);
            }
        }
    }
    
    sum = sum * 2 * area/(double)(N*N); // |j| = 2A
    
    return sum;
}

double smooth_image::sum_over_tri(std::vector<Vec2> tris)
{
    return get_sum_on_tri_template<double>
    (tris, std::function<double(Vec2)>([this](Vec2 p)
                                      {
                                          return get_value_f(p[0], p[1]);
                                      }
                                      ));
}

double smooth_image::get_variation_tri(std::vector<Vec2> tris, double c_mean)
{
    ci_temp = c_mean;
    return get_sum_on_tri_template<double>
    (tris, std::function<double(Vec2)>([this](Vec2 p)
                    {
                        return std::pow((get_value_f(p[0], p[1]) - ci_temp), 2);
                    }
                                       ));
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
    if (x < 0 or x >= _core_img.width()
        or y < 0 or y >= _core_img.height())
    {
        return;
    }
    
    static int height = _core_img.height();
    _core_img(x,height-1-y) = v;
}

Eigen::VectorXd smooth_image::reshape_to_vector()
{
//    Eigen::Map<Eigen::VectorXd> mapv(_core_img.data(), _core_img.width()*_core_img.height());
    // row major to column major
    Eigen::VectorXd mapv(_core_img.width()*_core_img.height());
    for (int c = 0; c < _core_img.width(); c++)
    {
        for (int r = 0; r < _core_img.height(); r++)
        {
            mapv(c*_core_img.height() + r) = _core_img(c,r);
        }
    }

    
    return mapv;
}

void smooth_image::display_list(std::vector<std::shared_ptr<smooth_image>> imgs)
{
    cimg_library::CImgList<double> imglist;
    
    for (auto p : imgs)
    {
        imglist.push_back(p->_core_img);
    }
    
    imglist.display();
}

void smooth_image::update(Eigen::VectorXd prob)
{
    // column major to row major
//    std::memcpy(_core_img.data(), prob.data(), prob.size()*sizeof(double));
    for (int c = 0; c < _core_img.width(); c++)
    {
        for (int r = 0; r < _core_img.height(); r++)
        {
            _core_img(c,r) = prob(c*_core_img.height() + r);
        }
    }
}

void smooth_image::blur(std::vector<std::shared_ptr<smooth_image>> imgs)
{
    for(auto & im : imgs)
    {
        im->_core_img.blur(1.0);
    }
}

void smooth_image::area_normalization
    (std::vector<std::shared_ptr<smooth_image>> imgs, std::vector<double> area)
{
    // Area normalization; from Vedrana
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

void smooth_image::from_buffer(double *buf, int w, int h)
{
    _core_img = CImg_class(w, h, 1, 1);
    for (int c = 0; c < _core_img.width(); c++)
    {
        for (int r = 0; r < _core_img.height(); r++)
        {
            _core_img(c,r) = buf[c*_core_img.height() + r];
        }
    }
}

void smooth_image::averaging(const std::vector<std::shared_ptr<smooth_image>> imgs)
{

    
    std::vector<double> color_maps = {1.0, 0.0, 0.5, 0.2, 0.8};
    
    _core_img.fill(0.0);
    for (int i = 0; i < imgs.size(); i++)
    {
        auto iptr = imgs[i];
        _core_img = _core_img + iptr->_core_img*color_maps[i];;
    }

//    _core_img.normalize(0.0, 1.0);
    
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
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, _core_img.width(), _core_img.height(), 0, GL_RGB, GL_UNSIGNED_BYTE, texture_buf);
    
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    
    free(texture_buf);
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