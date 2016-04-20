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
    void draw_probability();
    void draw_test_coord();
    
public:
    texture_segment();
    ~texture_segment();
    
    void update_probability();
    
    void update_dsc();
    
    void init(); // Load image and construct dictionary
    void init_dsc_phases();
    
    void show_all_probablity();
    
    void draw_debug();
private:
    // Deform interface
    void compute_probability_forces();
    void compute_curvature_force();
    
    // Deform DSC
    void displace_dsc();
    
    // Relabel trinagle
    void optimize_label();
    
private: // For debugging
    struct tri_variation
    {
        int phase;
        double variation;
    };
    HMesh::FaceAttributeVector<tri_variation> _tri_variation_debug;
    
public:
    dsc_sharedptr _dsc;
    // The original image
    std::shared_ptr<smooth_image> _origin_img;
    
    // Dictionary map
    std::unique_ptr<texture::dictionary> _dict;
    
    // Labeled image. Convert from DSC
    std::vector<std::shared_ptr<smooth_image>> _labeled_imgs;
    std::shared_ptr<smooth_image> _labeled_over_lay_img;
    
    // Probability
    std::vector<std::shared_ptr<smooth_image>> _probability_imgs;
    smooth_image_ptr _probability_over_lay_img;
};


#endif /* texture_segment_hpp */
