//
//  setting_file.cpp
//  DSC_texture_seg
//
//  Created by Tuan Nguyen Trung on 4/11/16.
//  Copyright © 2016 Asger Nyman Christiansen. All rights reserved.
//

#include <stdio.h>

#include "setting_file.h"


setting test =
{
    "DATA/test.png" // name
    ,{ // Initialization
        { // Phase 0
            {Vec2(100,100), 50}
        }
    }
    , 25.0
};

setting setting_file = test;