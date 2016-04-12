//
//  texture_segment.cpp
//  DSC_texture_seg
//
//  Created by Tuan Nguyen Trung on 4/7/16.
//  Copyright © 2016 Asger Nyman Christiansen. All rights reserved.
//

#include "texture_segment.hpp"
#include "setting_file.h"

texture_segment::texture_segment()
{
    
}

texture_segment::~texture_segment()
{
    
}

void texture_segment::update_dsc()
{
    _dsc->deform();
}

void texture_segment::init()
{
    // Load image
    _origin_img = std::shared_ptr<smooth_image>(new smooth_image);
    _origin_img->load_image(setting_file._image_name);
    
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