//
//  setting_file.h
//  DSC_texture_seg
//
//  Created by Tuan Nguyen Trung on 4/11/16.
//  Copyright © 2016 Asger Nyman Christiansen. All rights reserved.
//

#ifndef setting_file_h
#define setting_file_h

#include <stdlib.h>
#include <iostream>
#include <string>
#include <map>
#include <fstream>
#include <vector>
#include "define.h"

struct init_circle
{
    Vec2 _center;
    double _radius;
    
    bool is_in_circle(Vec2 pt){ return (pt-_center).length() < _radius; }
    bool is_in_circle(std::vector<Vec2> pts)
    {
        for(auto p: pts){
            if(!is_in_circle(p))
                return false;
        }
        return true;
    }
    
    Vec2 project_to_circle(Vec2 pt)
    {
        return _center + (pt - _center)*( _radius / (pt - _center).length());
    }
};

class setting
{
public:
    setting();
    ~setting(){};
    
    std::string _image_name; // Image to segment
    std::vector<std::vector<init_circle>> _circle_inits; // Initialization by circle
    
    bool _b_color;
    double batch_size;
    double branching_factor;
    double num_training_patch;
    double num_layer;
    bool normalize;
    
    double alpha;
    double edge_split_thres; // The smaller, the easier for splitting
    double face_split_thres; // The smaller, the eaiser for relabeling
    double min_edge_length; // DSC parammeter

    double dt;
    double dsc_discretization;

    
private:
    
    void load_raden(); // randen15.png image
    void load_synthetic1(); // test_C.png
    void load_test_A1();
    void load_leopard();
    void load_flower();
    
    void trick_border_image();
};

extern setting setting_file;

#endif /* setting_file_h */
