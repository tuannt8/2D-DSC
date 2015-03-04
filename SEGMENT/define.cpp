//
//  define.cpp
//  DSC
//
//  Created by Tuan Nguyen Trung on 2/20/15.
//  Copyright (c) 2015 Asger Nyman Christiansen. All rights reserved.
//

#include <stdio.h>
#include "define.h"

int debug_num[10] = {-1};


/*********************************************************************/
/* Different parameters
 */
#define MULTIPLE_BMP



/************************/
/* H.bmp
 */
#ifdef H_BMP // H.bmp

dynamics_param g_param;
std::string IMAGE_NAME = "H.bmp";
double DISCRETIZE_RES = 12;

/************************/
/* square.bmp
 */
#elif defined SQUARE_BMP
dynamics_param g_param = dynamics_param(0., 0., 0.0001, 50);

std::string IMAGE_NAME = "square.bmp";
double DISCRETIZE_RES = 8;//11.0


/************************/
/* multiple.bmp
 */
#elif defined MULTIPLE_BMP

dynamics_param g_param = dynamics_param(0., 0., 0.0001, 50);    ;
std::string IMAGE_NAME = "multiple.bmp";
double DISCRETIZE_RES = 15;//11.0

/************************/
/* multiple_noise.bmp
 */
#elif defined MULTIPLE_NOISE_BMP

dynamics_param g_param;
std::string IMAGE_NAME = "multiple_noise.bmp";
double DISCRETIZE_RES = 15;//11.0

/************************/
/* square_small.bmp
 */
#elif defined SQUARE_SMALL_BMP

dynamics_param g_param;
std::string IMAGE_NAME = "square_small.bmp";
double DISCRETIZE_RES = 15;//11.0

#else

// Should not be here

#endif
