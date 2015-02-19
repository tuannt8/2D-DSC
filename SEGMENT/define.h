//
//  define.h
//  DSC
//
//  Created by Tuan Nguyen Trung on 2/11/15.
//  Copyright (c) 2015 Asger Nyman Christiansen. All rights reserved.
//

#ifndef DSC_define_h
#define DSC_define_h
#include "DSC.h"

// Data directory
#ifdef WIN32

#else
#define DATA_PATH "../../../DATA/"
#define LOG_PATH "../../../LOG/"
#endif



typedef DSC2D::vec2 Vec2;
typedef DSC2D::DeformableSimplicialComplex dsc_obj;
typedef dsc_obj::node_key Node_key;
typedef std::vector<Vec2> Vec2_array;

extern int debug_num[10];

#define H_BMP

#pragma mark - Difference image
#ifdef H_BMP // H.bmp
    struct dynamics_param{
        double alpha = 0.00; // Second derivative. Keep the curve short
        double beta = 0.0; // Forth derivative. Keep the curve straight
        double gamma = 0.001; // External force scale
        double mass = 50;
    };
    #define IMAGE_NAME "H.bmp"
#elif SQUARE_BMP
    struct dynamics_param{
        double alpha = 0.00; // Second derivative. Keep the curve short
        double beta = 0.0; // Forth derivative. Keep the curve straight
        double gamma = 0.005; // External force scale
        double mass = 50;
    };
    #define IMAGE_NAME "square.bmp"
#endif

#endif // File protection
