//
//  image.h
//  DSC
//
//  Created by Tuan Nguyen Trung on 2/18/15.
//  Copyright (c) 2015 Asger Nyman Christiansen. All rights reserved.
//

#ifndef __DSC__image__
#define __DSC__image__

#include <stdio.h>
#include "define.h"
#include "CImg.h"

typedef unsigned char BYTE;
#define MAX_BYTE 255

class image: public cimg_library::CImg<BYTE>{
    
public:
    void load_image(std::string const file_path);
    // Draw in OpenGL coordinate
    void draw_image(int window_width);
    // 0 - 1.0
    double get_intensity_d(int x, int y);
    // 0 - 255
    int get_intensity_i(int x, int y);
    // total intensity inside a triangle
    int get_triangle_intensity_count(Vec2_array tris, int *nb_pixel = nullptr);
    
    Vec2 size(){return Vec2(width(), height());}
};

#endif /* defined(__DSC__image__) */
