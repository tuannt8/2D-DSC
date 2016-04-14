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

setting leopard =
{
    "DATA/leopard.png"
    ,{ // Initialization
        { // Phase 0
            {Vec2(100,100), 20}
        }
    }
    , 25.0 // DSC discretization
};

setting circle =
{
    "DATA/circle.png"
    ,{ // Initialization
        { // Phase 0
            {Vec2(100,60), 20}
        }
    }
    , 10.0 // DSC discretization
};

setting randen =
{
    "DATA/randen14.png"
    ,{ // Initialization
        { // Phase 0
            {Vec2(50,128), 20}
        }
        ,{ // Phase 1
            {Vec2(220,128), 20}
        }
        ,{ // Phase 2
            {Vec2(128,50), 20}
        }
        ,{ // Phase 3
            {Vec2(128,220), 20}
        }
        ,{ // Phase 4
            {Vec2(138,138), 20}
        }
    }
    , 20.0 // DSC discretization
};


setting setting_file = randen;