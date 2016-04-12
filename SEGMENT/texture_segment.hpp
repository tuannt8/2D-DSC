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
    void draw_dictionary();
public:
    texture_segment();
    ~texture_segment();
    
    void update_probability();
    
    void update_dsc();
    
    void init(); // Load image and construct dictionary
    void init_dsc_phases();
    
public:
    dsc_sharedptr _dsc;
    // The original image
    std::shared_ptr<smooth_image> _origin_img;
    
    std::unique_ptr<texture::dictionary> _dict;
    std::shared_ptr<smooth_image> _dict_over_lay_img;
public: // Get shared data
    
    std::vector<std::shared_ptr<smooth_image>> _prob_imgs;
};


#endif /* texture_segment_hpp */
