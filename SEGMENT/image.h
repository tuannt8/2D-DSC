//
//  image.h
//  DSC
//
//  Created by Tuan Nguyen Trung on 2/18/15.
//

#ifndef __DSC__image__
#define __DSC__image__

#include <stdio.h>
#include "define.h"
#include "CImg.h"

typedef unsigned char BYTE;
#define MAX_BYTE 255

/*
 Image 
 Dimension: 
 y ^
   |
  0|------>
          x (width)
 Scale: 0-1
 */

class image: public cimg_library::CImg<BYTE>{
private:
    // Row order storage idx = y * width + x
    Vec2_array gradient_;
    
private:
    void compute_gradient();

public:
    void load_image(std::string const file_path);
    // Draw in OpenGL coordinate
    void draw_image(int window_width);
    void draw_grad(int window_width);
    
    // double: 0 - 1.0
    double get_intensity(int x, int y);

    // total intensity inside a triangle
    void get_tri_intensity(Vec2_array tris, int * total_pixel, double * total_intensity);
    
    Vec2 size(){return Vec2(width(), height());}
    
    Vec2 grad(int x, int y);
};

#endif /* defined(__DSC__image__) */
