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
//    std::string sentence = "I have a feeling";
//    std::istringstream iss(sentence);
//    std::string buf;
//    while (iss >> buf)
//        std::cout << buf << std::endl;
//    
//    return 0;
    
    interface ui(argc, argv);
#ifndef TEST_PROBABILITY
    glutMainLoop();
#endif
    return 0;
}
