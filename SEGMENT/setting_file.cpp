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

using namespace std;
setting setting_file;

setting::setting()
{
    static int inited = 0;
    inited ++;
    assert(inited <= 1);
    
    
    batch_size = 5;
    branching_factor = 7;
    num_training_patch = 20000;
    num_layer = 4;
    normalize = true;
    
    alpha = 0.1;
    edge_split_thres = 0.05; // The smaller, the easier for splitting
    face_split_thres = 0.01; // The larger, the eaiser for relabeling
    min_edge_length = 5.0; // DSC parammeter
    
    update_prob_frequency = 50;
    adapt_mesh_frequency = 20;

    dt = 1.0;
    dsc_discretization = 25.0;

    _b_color = false;
    _bRelabel = true;
    _bTrickBorder = true;
    
    border_length = 0.0;
    
    load_test_case(15);

    
    if(_bTrickBorder)
        trick_border_image();
}

void setting::load_test_case(int idx)
{
    switch (idx) {
        case 1:
            load_leopard();
            break;
        case 2:
            load_flower();
            break;
        case 3:
            load_star_fish();
            break;
        case 4:
            load_tiger();
            break;
        case 5:
            load_tiger_group();
            break;
        case 6:
            load_tortoise();
            break;
        case 7:
            load_test_A1();
            break;
        case 8:
            load_test_A2();
            break;
        case 9:
            load_test_B1();
            break;
        case 10:
            load_test_B2();
            break;
        case 11:
            load_synthetic1();
            break;
        case 12:
            load_raden();
            break;
        case 13:
            load_corn_13();
            break;
        case 14:
            load_kanguru_14();
            break;
        case 15:{ // leopard stand
            _image_name = "DATA/test_images/Tuan/134035.png";
            _circle_inits ={{{Vec2(184, 115), 30}}};
            batch_size = 3;
            branching_factor = 8;
            num_training_patch = 50000;
            num_layer = 4;
            _b_color = true;
            _bRelabel = false;
            _bTrickBorder = false;
            face_split_thres = 0.01;
            min_edge_length = 10.0;
            alpha = 0.5; // relabeling? 1.5 : 0.5
        } break;
        case 16:{ // zebra
            _image_name = "DATA/test_images/Tuan/253027.png";
            _circle_inits ={{{Vec2(161, 151), 30}}};
            _bRelabel = false;
            if(_bRelabel)
            {
                batch_size = 15;
                branching_factor = 7;
                num_training_patch = 50000;
                num_layer = 4;
                _b_color = true;

                _bTrickBorder = false;
                face_split_thres = 0.01;
                min_edge_length = 6.0;
                alpha = 3;
            }else{
                batch_size = 15;
                branching_factor = 7;
                num_training_patch = 50000;
                num_layer = 4;
                _b_color = true;
                
                _bTrickBorder = false;
                face_split_thres = 0.1;
                min_edge_length = 4.0;
                alpha = 2.5;
                
                dt = 1;
            }
        }break;
        case 17:{//raden
            _image_name = "DATA/test_images/Tuan/texture06_16.png";
            _circle_inits ={{{Vec2(145, 30), 30}}, {{Vec2(397, 29), 30}}, {{Vec2(39, 137), 30}},
            {{Vec2(146, 135), 30}}, {{Vec2(253, 133), 30}}, {{Vec2(333, 140), 30}},
            {{Vec2(481, 159), 30}}, {{Vec2(140, 252), 30}}, {{Vec2(296, 253), 30}},
            {{Vec2(38, 377), 30}}, {{Vec2(136, 372), 30}}, {{Vec2(254, 376), 30}},
            {{Vec2(365, 365), 30}}, {{Vec2(484, 372), 30}}, {{Vec2(134, 478), 30}}
//            ,{{Vec2(380, 479), 30}}
            };
            
            batch_size = 19;
            branching_factor = 7;
            num_training_patch = 50000;
            num_layer = 4;
            _b_color = false;
            _bTrickBorder = false;
        }break;
        default:
            break;
    }
}

void setting::load_test()
{
    _image_name = "DATA/test_images/253027.png";
    batch_size = 5;
    branching_factor = 7;
    num_training_patch = 50000;
    num_layer = 5;
    normalize = true;
    min_edge_length = 5;
    
    _circle_inits = { // Initialization
        { // Phase 0
            {Vec2(105,148), 30}
        }
    };
    
    _b_color = true;
    _bTrickBorder = false;
}

void setting::load_kanguru_14()
{
    _image_name = "DATA/test_images/Tuan/69020.png";
    _circle_inits =
    {
        {
            {Vec2(209, 200), 30}
        }
    };
    
    _b_color = true;
    _bTrickBorder = false;

    
    _bRelabel = true;
    
    if (_bRelabel)
    {
        batch_size = 5;
        branching_factor = 7;
        num_training_patch = 50000;
        num_layer = 5;
        face_split_thres = 0.01;
        min_edge_length = 10.0;
        
        alpha = 2.0;
    }
    else{
        batch_size = 5;
        branching_factor = 7;
        num_training_patch = 50000;
        num_layer = 5;
        face_split_thres = 0.01;
        min_edge_length = 10.0;
        
        alpha = 2.0;
    }
}

void setting::load_corn_13()
{
    _image_name = "DATA/test_images/Tuan/58060.png";
    _circle_inits = {{{Vec2(209, 165), 30}}};
    
    _b_color = true;
    _bTrickBorder = false;

    
    _bRelabel = false;
    
    if (_bRelabel)
    {
        batch_size = 19;
        branching_factor = 7;
        num_training_patch = 20000;
        num_layer = 4;
        alpha = 16.0;
    }
    else
    {
        batch_size = 15;
        branching_factor = 7;
        num_training_patch = 20000;
        num_layer = 4;
        alpha = 1;
    }
}

void setting::load_raden()
{
    _image_name = "DATA/test_images/randen15.png";
//    _circle_inits = { // Initialization
//        { // Phase 0
//            {Vec2(50,128), 20}
//        }
//        ,{ // Phase 1
//            {Vec2(220,128), 20}
//        }
//        ,{ // Phase 2
//            {Vec2(128,50), 20}
//        }
//        ,{ // Phase 3
//            {Vec2(128,220), 20}
//        }
//        ,{ // Phase 4
//            {Vec2(138,138), 20}
//        }
//    };
    
    _circle_inits =
    {
        { // Phase 1
            {Vec2(128, 32), 30} // (center), radius. Can be more than one circle
        }
        ,{ // Phase 2
            {Vec2(128, 128), 30}
        }
        ,{
            {Vec2(128, 225), 30}
        }
        ,{
            {Vec2(32, 128), 30}
        }
    };
    
    batch_size = 15;
    branching_factor = 6;
    num_training_patch = 50000;
    num_layer = 4;
    
    
    _bTrickBorder = false;
    _bRelabel = true;
    alpha = 0.3;
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
    
    batch_size = 15;
    
    face_split_thres = 0.02;
    edge_split_thres = 0.1;
    
    alpha = 0.0;
    
    dt = 3;
}

void setting::load_leopard()
{
    dsc_discretization = 25;
    _image_name = "DATA/test_images/leopard.png";
    _b_color = true;
    _circle_inits = {
        {
            {Vec2(169,169), 25}
        }
    };
    alpha = 4;
    
    branching_factor = 7;
    num_training_patch = 50000;
    num_layer = 5;
    normalize = true;
    
    batch_size = 3;
    dt = 0.3;
    
    edge_split_thres = 0.1;
    face_split_thres = 0.1;
    min_edge_length = 10;
    
    _bTrickBorder = false;
    _bRelabel = true;
}
// 4
void setting::load_tiger()
{
    _image_name = "DATA/test_images/108073.png";
    _circle_inits = {
        {
            {Vec2(215,146), 25}
        }
    };
    batch_size = 5;
    
    branching_factor = 5;
    num_training_patch = 10000;
    num_layer = 4;
    normalize = true;
    
    _bTrickBorder = false;
    _b_color = true;
    _bRelabel = false;
}

// 3
void setting::load_star_fish()
{
    _image_name = "DATA/test_images/12003.png";
    _circle_inits = {
        {
            {Vec2(249,201), 25}
        }
    };
    batch_size = 3;
    
    branching_factor = 7;
    num_training_patch = 10000;
    num_layer = 4;
    
    alpha = 0.1;
    edge_split_thres = 0.2;
    face_split_thres = 0.1;
    min_edge_length = 5;
    dt = 1;
    
    _bTrickBorder = false;
    _b_color = true;
    _bRelabel = true;
    
    alpha = 16.0;
    
    update_prob_frequency = 50;
    adapt_mesh_frequency = 20;
}

// 5
void setting::load_tiger_group()
{
    _image_name = "DATA/test_images/105053.png";
    _circle_inits = {
        {
            {Vec2(201,119), 30}
        }
    };
    batch_size = 9;

    branching_factor = 7;
    num_training_patch = 50000;
    num_layer = 4;
    
    alpha = 0.4;
    edge_split_thres = 0.01;
    face_split_thres = 0.0001;
    min_edge_length = 10;
    dt = 1;
    
    _bTrickBorder = false;
    _b_color = true;
    _bRelabel = false;
}

// 6
void setting::load_tortoise()
{
    _image_name = "DATA/test_images/tortoise.png";
    _circle_inits = {
        {
            {Vec2(216,113), 50}
        }
//        ,{
//            {Vec2(424,35),20}
//        }
//        ,{
//            {Vec2(23,64),20}
//        }
    };
    batch_size = 3;
    
    _bRelabel = false;
    _bTrickBorder = false;
    if (_bRelabel)
    {
        face_split_thres = 0.01;
        alpha = 4.0;
        min_edge_length = 20;
    }
    else{
        face_split_thres = 0.01;
        alpha = 4.0;
        min_edge_length = 20;
        dt = 3;
    }
}

// 8
void setting::load_test_A2()
{
    _image_name = "DATA/test_images/test_A2.png";
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
    
    batch_size = 9;
    
    min_edge_length = 20.;
    edge_split_thres = 0.1;
    face_split_thres = 0.008;
    alpha = 1;
    
    _bTrickBorder = true;
}

// 8
void setting::load_test_B1()
{
    _image_name = "DATA/test_images/test_B1.png";
}

// 9
void setting::load_test_B2()
{
    _image_name = "DATA/test_images/test_B2.png";
}

void setting::load_flower()
{
    _image_name = "DATA/test_images/flower.png";
    _b_color = true;
    _circle_inits = {
        {
            {Vec2(220,150), 50}
        }
    };
    
    
    min_edge_length = 3;
    batch_size = 5;
    dt = 3;
    face_split_thres = 0.02;
    alpha = 0.1;
    
    branching_factor = 7;
    num_training_patch = 10000;
    num_layer = 4;
    
    _bTrickBorder = false;
    _bRelabel = false;
    
    dsc_discretization = 25.0;
    
    update_prob_frequency = 50;
    adapt_mesh_frequency = 20;
}

void setting::load_test_A1()
{
    dsc_discretization = 30;
    _image_name = "DATA/test_images/test_A1.png";
//    _circle_inits = {
//        {
//            {Vec2(95,504), 50}
//        }
//        ,{
//            {Vec2(504,95),50}
//        }
//        ,{
//            {Vec2(250,350),50}
//        }
//        ,{
//            {Vec2(400,300),50}
//        }
//    };
    _circle_inits = {
        {
            {Vec2(85,504), 25}
        }
        ,{
            {Vec2(115,504),25}
        }
        ,{
            {Vec2(504,95),25}
        }
        ,{
            {Vec2(534,125),25}
        }
        ,{
            {Vec2(250,350),50}
        }
        ,{
            {Vec2(400,300),50}
        }
    };
    batch_size = 11;
    
    min_edge_length = 20.;
    edge_split_thres = 0.1;
    face_split_thres = 0.1;
    alpha = 0.3;
    
    _bRelabel = false;
//    _bTrickBorder = false;
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
            
            border_length = border;
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
            
            border_length = border;
        }
    }
    catch (const std::exception& e)
    {
        std::cout << e.what();
        exit(1);
    }
}

#define _COLOR "color"
#define _PATH_SIZE "patch size"
#define _BRANCHING_FACTOR "branching factor"
#define _NUM_TRAINING_PATCH "Number of training patches"
#define _NUM_LAYER  "Number of layer"
#define _NORMALIZE  "normalize"

#define _ALPHA          "alpha"
#define _EDGE_SPLIT     "Edge split thres"
#define _FACE_RELABEL   "Face relabel thres"
#define _MIN_EDGE       "min edge"
#define _PROB_FREQUENCY "update probability frequency"
#define _ADAPT_MESH_FREQUENCY "adapt mesh frequency"


void setting::load_setting_from_file(string path)
{
    std::ifstream f(path);
    map<string, string> setting_map;
    if (f.is_open())
    {
        std::string line;
        while (f >> line)
        {
            if (line.substr(0,2) == "//")
            {
                continue;
            }
            string left = line.substr(0, line.find(":"));
            string right = line.substr(left.size(), line.size() - left.size());
            setting_map.insert(make_pair(left, right));
        }
        f.close();
        
        
    }
    else{
        throw "Setting file not exit";
    }
}