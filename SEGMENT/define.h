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
#include <memory.h>

#define cimg_use_png
// Define 'cimg_display' to: '0' to disable display capabilities.
//                           '1' to use the X-Window framework (X11).
//                           '2' to use the Microsoft GDI32 framework.
#define cimg_display 1

#pragma clang diagnostic push
// in reality, you will likely need to disable *more* than Wmultichar
#pragma clang diagnostic ignored "-Wmultichar"
#include "CImg.h"
#pragma clang diagnostic pop

// Data directory
#ifdef WIN32

#else
#define DATA_PATH "./DATA/"
#define LOG_PATH "./LOG/"
#endif

#define PI_V1 3.14159

typedef unsigned char BYTE;
#define MAX_BYTE 255

/*
 * Two phase synthetics image
 */
// Coefficient
#define IMAGE_PATH "lion.png"
// Mesh control
#define DISCRETIZE_RES 25.0
#define SMALLEST_SIZE   4
// Adaptive mesh
#define SPLIT_FACE_COEFFICIENT  0.2 // split thres = coe*(1-coe)*(ci-cj)^2
#define SPLIT_EDGE_COEFFICIENT 3    // Split thres = coe*(ci-cj)^2
// Mumford
#define ALPHA 0.01  // internal force
#define BETA    1.0 // external force

#define DT_ 2
//#define ADD_NOISE
#define STABLE_MOVE 0.1


using std::vector;
using std::cout;
using std::endl;

/*********************************************************************/
/* Type def
 */
typedef DSC2D::vec2 Vec2;
typedef DSC2D::vec3 Vec3;
typedef DSC2D::DeformableSimplicialComplex dsc_obj;
typedef std::shared_ptr<DSC2D::DeformableSimplicialComplex> dsc_sharedptr;
typedef dsc_obj::node_key Node_key;
typedef dsc_obj::face_key Face_key;
typedef dsc_obj::edge_key Edge_key;
typedef std::vector<Vec2> Vec2_array;

struct dynamics_param{
    dynamics_param(){}
    dynamics_param(double a, double b, double m){alpha = a; beta = b; mass = m;};
    
    double alpha = 1.0; // Curvature
    double beta = 1.0; // Forth derivative. Keep the curve straight
    double mass = 10.0; // Display scale
    
    std::map<int, double> mean_intensity;
    
    std::vector<bool> bDisplay;
} ;

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


#endif // File protection
