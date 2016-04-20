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

struct setting
{
    std::string _image_name; // Image to segment
    std::vector<std::vector<init_circle>> _circle_inits; // Initialization by circle
    double dsc_discretization;
    
//    double variation_relabel_thres;
};

extern setting setting_file;

#endif /* setting_file_h */
