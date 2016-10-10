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
#define TEST_BMP



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
#elif defined MM_BMP
dynamics_param g_param(1,1,0.01);

std::string IMAGE_NAME = "mm.bmp";
double DISCRETIZE_RES = 20;//11.0

/************************/
/* test.bmp
 */
#elif defined TEST_BMP
dynamics_param g_param(1.0,1.0,1.);

std::string IMAGE_NAME = "tuesday/im_1.png";
double DISCRETIZE_RES =15;//11.0

/************************/
/* arrow.bmp
 */
#elif defined ARROW_BMP

dynamics_param g_param(0.1, 1, 10);
std::string IMAGE_NAME = "arrow.bmp";
double DISCRETIZE_RES = 11;//11.0

/************************/
/* multiple.bmp
 */
#elif defined MULTIPLE_BMP

dynamics_param g_param(1, 1, 10);
std::string IMAGE_NAME = "multiple.bmp";
double DISCRETIZE_RES = 22;//11.0

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

/************************/
/* square_small.bmp
 */
#elif defined FOX_BMP

dynamics_param g_param;
std::string IMAGE_NAME = "fox.bmp";
double DISCRETIZE_RES = 10;//11.0

#else

// Should not be here

#endif
