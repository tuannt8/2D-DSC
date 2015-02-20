//
//  helper.h
//  DSC
//
//  Created by Tuan Nguyen Trung on 2/12/15.
//  Copyright (c) 2015 Asger Nyman Christiansen. All rights reserved.
//

#ifndef __DSC__helper__
#define __DSC__helper__

#include <stdio.h>
#include "define.h"

namespace helper_t {
    // Check point inside triangle
    float sign(Vec2 p1, Vec2 p2, Vec2 p3);
    bool is_point_in_tri(Vec2 p, std::vector<Vec2> const & pts);
    bool is_point_in_tri(Vec2 p, Vec2 t1, Vec2 t2, Vec2 t3);
    
    class autoColor{
    public:
        autoColor(){};
        ~autoColor(){};
        int index = 0;
        Vec3 next();
    };
}

#endif /* defined(__DSC__helper__) */
