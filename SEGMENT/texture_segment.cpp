//
//  texture_segment.cpp
//  DSC_texture_seg
//
//  Created by Tuan Nguyen Trung on 4/7/16.
//  Copyright © 2016 Asger Nyman Christiansen. All rights reserved.
//

#include "texture_segment.hpp"
#include "setting_file.h"

using namespace std;
#define INT_BIG 99999

texture_segment::texture_segment()
{
    
}

texture_segment::~texture_segment()
{
    
}

void get_bounding_box(const vector<Vec2> & pts , CGLA::Vec2i & ld, CGLA::Vec2i & ru)
{
    ld = CGLA::Vec2i(INT_BIG);
    ru = CGLA::Vec2i(-INT_BIG);
    
    for (auto & p : pts)
    {
        ld[0] = (int)min((double)ld[0], p[0]);
        ld[1] = (int)min((double)ld[1], p[1]);
        ru[0] = (int)max((double)ru[0], p[0]);
        ru[1] = (int)max((double)ru[1], p[1]);
    }
}

bool intersect_tri(const vector<Vec2> & pts, double x, double & miny, double &maxy)
{
    miny = INFINITY;
    maxy = -INFINITY;
    
    static int idx[] = {1, 2, 0};
    
    for (int i = 0; i < 3; i++)
    {
        auto p0 = pts[i];
        auto p1 = pts[idx[i]];
        
        {
            double t = (x+0.1-p0[0]) / (p1[0] - p0[0]);
            if (t >= 0 and t <= 1)
            {
                double y = p0[1] + t*(p1[1] - p0[1]);
                miny = min(miny, y);
                maxy = max(maxy, y);
            }
        }
        {// To make sure that we don't miss any intersection. Optimize later.
            double t = (x+0.9-p0[0]) / (p1[0] - p0[0]);
            if (t >= 0 and t <= 1)
            {
                double y = p0[1] + t*(p1[1] - p0[1]);
                miny = min(miny, y);
                maxy = max(maxy, y);
            }
        }
    }
    
    return miny != INFINITY;
}

void texture_segment::update_probability()
{
    for (auto im : _prob_imgs)
    {
        im->fill(0.5);
    }
    
    for(auto tri : _dsc->faces())
    {
        int phase = _dsc->get_label(tri);
        if (phase == 0)
        {
            continue;
        }
        
        auto pts = _dsc->get_pos(tri);
        CGLA::Vec2i ld, ru;
        get_bounding_box(pts, ld, ru);
        
        // The triangle is convect. It is easy
        double min_y, max_y;
        for (int x = floor(ld[0]); x < ceil(ru[0]); x++)
        {
            // Find intersection
            if (intersect_tri(pts, x, min_y, max_y))
            {
                // update dictionary
                for (int y = floor(min_y); y < ceil(max_y); y++)
                {
                    _prob_imgs[phase-1]->set_value(x,y,1.0);
                }
            }
        }
    }
    
    _dict_over_lay_img->averaging(_prob_imgs);
}

void texture_segment::draw_dictionary()
{
    if (_dict_over_lay_img)
    {
        _dict_over_lay_img->draw_image();
    }
}

void texture_segment::update_dsc()
{
    update_probability();
    _dsc->deform();
}

void texture_segment::init()
{
    // Load image
    _origin_img = std::shared_ptr<smooth_image>(new smooth_image);
    _origin_img->load_image(setting_file._image_name);
    
    // Update the probability
    _prob_imgs = std::vector<std::shared_ptr<smooth_image>>
                        (setting_file._circle_inits.size(),
                         std::shared_ptr<smooth_image>(
                        new smooth_image(_origin_img->width(), _origin_img->height()))
                         );
    _dict_over_lay_img = std::shared_ptr<smooth_image>(
                        new smooth_image(_origin_img->width(), _origin_img->height())
                                         );
    // Construct dictionary
//    _dict = std::unique_ptr<texture::dictionary>
//                (new texture::dictionary(setting_file._image_name));
}

void texture_segment::init_dsc_phases()
{
    // Relabel
    auto circle_init = setting_file._circle_inits;
    for (auto tri : _dsc->faces())
    {
        auto pts = _dsc->get_pos(tri);
        // check if it belong to any circle
        for (int i = 0; i < circle_init.size(); i++)
        {
            auto & circles = circle_init[i];
            for (int j = 0; j < circles.size(); j++)
            {
                auto & circle = circles[j];
                if(circle.is_in_circle(pts)){
                    _dsc->update_attributes(tri, i+1);
                }
            }
        }
    }
    
    // Deform
    for (auto v:_dsc->vertices())
    {
        if (_dsc->is_interface(v))
        {
            auto pt = _dsc->get_pos(v);
            // check if it belong to any circle
            for (int i = 0; i < circle_init.size(); i++)
            {
                auto & circles = circle_init[i];
                for (int j = 0; j < circles.size(); j++)
                {
                    auto & circle = circles[j];
                    if(circle.is_in_circle(pt)){
                        auto pc = circle.project_to_circle(pt);
                        _dsc->set_destination(v, pc);
                    }
                }
            }
        }
    }
    _dsc->deform();
}