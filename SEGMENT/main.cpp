//
//  SEGMENT.cpp
//  DSC
//
//  Created by Tuan Nguyen Trung on 2/9/15.
//  Copyright (c) 2015 Asger Nyman Christiansen. All rights reserved.
//


#include "interface.h"
//#include "DSC.h"

int main(int argc, char ** argv) {
    interface ui(argc, argv);
#ifndef TEST_PROBABILITY
    glutMainLoop();
#endif
    return 0;
}
