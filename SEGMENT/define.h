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
#include <string>

// Data directory
#ifdef WIN32

#else
#define DATA_PATH "../../../DATA/"
#define LOG_PATH "../../../LOG/"
#endif

/*********************************************************************/
/* Type def
 */
typedef DSC2D::vec2 Vec2;
typedef DSC2D::vec3 Vec3;
typedef DSC2D::DeformableSimplicialComplex dsc_obj;
typedef dsc_obj::node_key Node_key;
typedef dsc_obj::face_key Face_key;
typedef std::vector<Vec2> Vec2_array;

struct dynamics_param{
    double alpha = 0.01; // Second derivative. Keep the curve short
    double beta = 0.01; // Forth derivative. Keep the curve straight
    double gamma = 0.001; // External force scale
    double mass = 50;
};

/*******
 Flags
 */

/*********************************************************************/
/* GLobal variable
 */
extern int debug_num[10];

// Image to load
extern std::string IMAGE_NAME;
// Dynamics parameter
extern dynamics_param g_param;
// Discretization
extern double DISCRETIZE_RES;


#endif // File protection
