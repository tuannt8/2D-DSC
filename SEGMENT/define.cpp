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

/************************/
/* GOMU:
 */
dynamics_param g_param(0.1,1,0.01);

std::string IMAGE_NAME = "Data/Gomu/TallHamster_x_135.png";
double DISCRETIZE_RES = 15;//11.0

/************************/
/* SOUND IMAGE
 */
//dynamics_param g_param(0.1,1,0.01);
//
//std::string IMAGE_NAME = "Data/sound_gap.png";
//double DISCRETIZE_RES = 25;//11.0


/************************/
/* Tetra Pak
 */
//dynamics_param g_param(0.1,1,0.01);
//
//std::string IMAGE_NAME = "Data/TetraPak/sample.png";
//double DISCRETIZE_RES = 40;//11.0

/************************/
/* square.bmp
 */
//dynamics_param g_param(0.1,1,0.1);
//
//std::string IMAGE_NAME = "mm.bmp";
//double DISCRETIZE_RES = 10;//11.0

/************************/
/* Brain image
 */
//dynamics_param g_param(0.1,1,0.01);
//
//std::string IMAGE_NAME = "brain_s.png";
//double DISCRETIZE_RES = 50;//11.0

/************************/
/* test.bmp
 */
//dynamics_param g_param(0.1, // alpha
//                       1.0, // beta
//                       1. // dt
//                       );
//
//std::string IMAGE_NAME = "test.png";
//double DISCRETIZE_RES = 8.0;//11.0

/************************/
/* test.bmp
 */
//#elif defined CHALK_TIFF
//dynamics_param g_param(1.0,1.0,1.);
//
//std::string IMAGE_NAME = "chalk.BMP";
//double DISCRETIZE_RES = 39;//11.0

/************************/
/* arrow.bmp
 */
//#elif defined ARROW_BMP
//
//dynamics_param g_param(0.1, 1, 10);
//std::string IMAGE_NAME = "arrow.bmp";
//double DISCRETIZE_RES = 22;//11.0

/************************/
/* multiple.bmp
 */
//#elif defined MULTIPLE_BMP
//
//dynamics_param g_param(1, 1, 10);
//std::string IMAGE_NAME = "multiple.bmp";
//double DISCRETIZE_RES = 22;//11.0

/************************/
/* multiple_noise.bmp
 */
//#elif defined MULTIPLE_NOISE_BMP
//
//dynamics_param g_param;
//std::string IMAGE_NAME = "multiple_noise.bmp";
//double DISCRETIZE_RES = 15;//11.0

/************************/
/* square_small.bmp
 */
//#elif defined SQUARE_SMALL_BMP
//
//dynamics_param g_param;
//std::string IMAGE_NAME = "square_small.bmp";
//double DISCRETIZE_RES = 15;//11.0

/************************/
/* square_small.bmp
 */
//#elif defined FOX_BMP
//
//dynamics_param g_param;
//std::string IMAGE_NAME = "fox.bmp";
//double DISCRETIZE_RES = 10;//11.0
