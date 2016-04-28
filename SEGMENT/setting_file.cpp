//
//  setting_file.cpp
//  DSC_texture_seg
//
//  Created by Tuan Nguyen Trung on 4/11/16.
//  Copyright © 2016 Asger Nyman Christiansen. All rights reserved.
//

#include <stdio.h>

#include "setting_file.h"
#include "define.h"


setting setting_file;

setting::setting()
{
     _b_color = false;
     batch_size = 15;
     branching_factor = 5;
     num_training_patch = 5000;
     num_layer = 4;
    
     alpha = 0.1;
     edge_split_thres = 0.05; // The smaller, the easier for splitting
     face_split_thres = 0.02; // The smaller, the eaiser for relabeling
     min_edge_length = 4; // DSC parammeter
    
     dsc_discretization = 25.0;
     edge_split_energy = 0.1;
     tri_split_energy = 0.1;
     dt = 1.0;
    
    
    
//    load_test_A1();
//    load_synthetic1();
//    load_raden();
    load_leopard();
    
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

void setting::load_leopard()
{
    dsc_discretization = 25;
    _image_name = "DATA/test_images/leopard.png";
    _b_color = true;
    _circle_inits = {
        {
            {Vec2(169,169), 50}
        }
    };
}

void setting::load_test_A1()
{
    dsc_discretization = 25;
    _image_name = "DATA/test_images/test_A1.png";
    _circle_inits = {
        {
            {Vec2(95,504), 50}
        }
        ,{
            {Vec2(504,95),50}
        }
        ,{
            {Vec2(250,350),50}
        }
        ,{
            {Vec2(400,300),50}
        }
    };

}
void setting::trick_border_image()
{
    try
    {
        if (setting_file._b_color)
        {
            // Create border on the image
            cimg_library::CImg<double> img_temp;
            img_temp.load(_image_name.c_str());
            
            // create bigger image
            int border = 0.05 * img_temp.width();
            cimg_library::CImg<double> newImg(img_temp.width() + 2*border,
                                              img_temp.height() + 2*border, 1, 3);
            newImg.fill(0.0, 0.0, 0.0);
            for (int i = 0; i < img_temp.width(); i++)
            {
                for (int j = 0; j < img_temp.height(); j++)
                {
                    newImg(i+border, j + border, 0) = img_temp(i,j, 0);
                    newImg(i+border, j + border, 1) = img_temp(i,j, 1);
                    newImg(i+border, j + border, 2) = img_temp(i,j, 2);
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
        else
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
    }
    catch (const std::exception& e)
    {
        std::cout << e.what();
        exit(1);
    }}