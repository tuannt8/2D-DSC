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

#define IMAGE_NAME "square1.bmp"

typedef DSC2D::vec2 Vec2;
typedef DSC2D::DeformableSimplicialComplex dsc_obj;
typedef dsc_obj::node_key Node_key;
typedef std::vector<Vec2> Vec2_array;

extern int debug_num[10];

#endif // File protection
