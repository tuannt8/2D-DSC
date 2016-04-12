//
//  texture_segment.hpp
//  DSC_texture_seg
//
//  Created by Tuan Nguyen Trung on 4/7/16.
//  Copyright © 2016 Asger Nyman Christiansen. All rights reserved.
//

#ifndef texture_segment_hpp
#define texture_segment_hpp

#include <stdio.h>
#include "texture.h"
#include "DSC.h"
#include "define.h"
#include <GL/glew.h>
#include "smooth_image.hpp"
#include "texture.h"

class texture_segment
{
public:
    texture_segment();
    ~texture_segment();
    
    void update_dsc();
    
    void init(); // Load image and construct dictionary
    void init_dsc_phases();
    
public:
    dsc_sharedptr _dsc;
    std::shared_ptr<smooth_image> _origin_img;
    
    std::unique_ptr<texture::dictionary> _dict;
public: // Get shared data
    void set_mesh(dsc_sharedptr dsc){_dsc = dsc;}
};


#endif /* texture_segment_hpp */
