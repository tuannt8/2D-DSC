//
//  setting_file.cpp
//  DSC_texture_seg
//
//  Created by Tuan Nguyen Trung on 4/11/16.
//  Copyright © 2016 Asger Nyman Christiansen. All rights reserved.
//

#include <stdio.h>

#include "setting_file.h"
#include "CImg.h"


setting setting_file;

setting::setting()
{
    load_synthetic1();
//    load_raden();
    
    trick_border_image();
}

void setting::load_raden()
{
    _image_name = "DATA/randen14.png";
    _circle_inits = { // Initialization
        { // Phase 0
            {Vec2(50,128), 20}
        }
        ,{ // Phase 1
            {Vec2(220,128), 20}
        }
        ,{ // Phase 2
            {Vec2(128,50), 20}
        }
        ,{ // Phase 3
            {Vec2(128,220), 20}
        }
        ,{ // Phase 4
            {Vec2(138,138), 20}
        }
    };
}

void setting::load_synthetic1()
{
    dsc_discretization = 25;
    _image_name = "DATA/test_images/test_C.png";
    _circle_inits = {
        {
            {Vec2(250,250), 50}
        }
        ,{
            {Vec2(250,350),25}
        }
        ,{
            {Vec2(400,250),25}
        }
        ,{
            {Vec2(550,250),50}
        }
    };
}

void setting::trick_border_image()
{
    try
    {
        // Create border on the image
        cimg_library::CImg<double> img_temp;
        img_temp.load(_image_name.c_str());
        
        // create bigger image
        int border = 0.05 * img_temp.width();
        cimg_library::CImg<double> newImg(img_temp.width() + 2*border,
                                          img_temp.height() + 2*border);
        newImg.fill(0.0);
        for (int i = 0; i < img_temp.width(); i++)
        {
            for (int j = 0; j < img_temp.height(); j++)
            {
                newImg(i+border, j + border) = img_temp(i,j);
            }
        }
        
        newImg.save("DATA/temp.png");
        
        _image_name = "DATA/temp.png";
        for (auto & init : _circle_inits)
        {
            for (auto & c : init)
            {
                c._center += Vec2(border, border);
            }
        }
    }
    catch (const std::exception& e)
    {
        std::cout << e.what();
        exit(1);
    }}