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
#define DATA_PATH "./DATA/"
#define LOG_PATH "./LOG/"
#endif

#define PI_V1 3.14159

///*
// * Two phase synthetics image
// */
//// Coefficient
//#define IMAGE_PATH "two_phase_hole.png"
//// Mesh control
//#define DISCRETIZE_RES 8.0
//#define SMALLEST_SIZE   30
//// Adaptive mesh
//#define SPLIT_FACE_COEFFICIENT  0.2 // split thres = coe*(1-coe)*(ci-cj)^2
//#define SPLIT_EDGE_COEFFICIENT 2    // Split thres = coe*(ci-cj)^2
//// Mumford
//#define ALPHA 0.1  // internal force
//#define BETA    1.0 // external force


///*
// * Sound
// */
//// Coefficient
//#define IMAGE_PATH "Data/sound_gap.png"
//// Mesh control
//#define DISCRETIZE_RES 20.0
//#define SMALLEST_SIZE   3.0
//// Adaptive mesh
//#define SPLIT_FACE_COEFFICIENT  0.08 // split thres = coe*(1-coe)*(ci-cj)^2
//#define SPLIT_EDGE_COEFFICIENT 3    // Split thres = coe*(ci-cj)^2
//// Mumford
//#define ALPHA 0.1  // internal force
//#define BETA    1.0 // external force

/*
 * Fuel cell
 */
// Coefficient
#define IMAGE_PATH "Data/fuel_cell/small_gap.png"
// Mesh control
#define DISCRETIZE_RES 20.0
#define SMALLEST_SIZE   2.0
// Adaptive mesh
#define SPLIT_FACE_COEFFICIENT  0.08 // split thres = coe*(1-coe)*(ci-cj)^2
#define SPLIT_EDGE_COEFFICIENT 1    // Split thres = coe*(ci-cj)^2
// Mumford
#define ALPHA 0.1  // internal force
#define BETA    1.0 // external force

using std::vector;
using std::cout;
using std::endl;

/*********************************************************************/
/* Type def
 */
typedef DSC2D::vec2 Vec2;
typedef DSC2D::vec3 Vec3;
typedef DSC2D::DeformableSimplicialComplex dsc_obj;
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
