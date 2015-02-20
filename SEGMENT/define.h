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

/*********************************************************************/
/* Type def
 */
typedef DSC2D::vec2 Vec2;
typedef DSC2D::vec3 Vec3;
typedef DSC2D::DeformableSimplicialComplex dsc_obj;
typedef dsc_obj::node_key Node_key;
typedef dsc_obj::face_key Face_key;
typedef std::vector<Vec2> Vec2_array;

/*********************************************************************/
/* GLobal variable
 */
extern int debug_num[10];

/*********************************************************************/
/* Different parameters
 */
#define MULTIPLE_BMP

#ifdef H_BMP // H.bmp
    struct dynamics_param{
        double alpha = 0.00; // Second derivative. Keep the curve short
        double beta = 0.0; // Forth derivative. Keep the curve straight
        double gamma = 0.001; // External force scale
        double mass = 50;
    };
    #define IMAGE_NAME "H.bmp"
    #define DISCRETIZE_RES 13.0

#elif defined SQUARE_BMP
    struct dynamics_param{
        double alpha = 0.00; // Second derivative. Keep the curve short
        double beta = 0.0; // Forth derivative. Keep the curve straight
        double gamma = 0.005; // External force scale
        double mass = 50;
    };
    #define IMAGE_NAME "square.bmp"
    #define DISCRETIZE_RES 15.0

#elif defined MULTIPLE_BMP
    struct dynamics_param{
        double alpha = 0.00; // Second derivative. Keep the curve short
        double beta = 0.0; // Forth derivative. Keep the curve straight
        double gamma = 0.001; // External force scale
        double mass = 50;
    };
    #define IMAGE_NAME "multiple.bmp"
    #define DISCRETIZE_RES 11.0

#elif defined SQUARE_SMALL_BMP
struct dynamics_param{
    double alpha = 0.00; // Second derivative. Keep the curve short
    double beta = 0.0; // Forth derivative. Keep the curve straight
    double gamma = 0.005; // External force scale
    double mass = 50;
};
#define IMAGE_NAME "square_small.bmp"
#define DISCRETIZE_RES 8.0

#else
#endif

#endif // File protection
