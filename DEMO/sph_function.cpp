//
//  sph_function.cpp
//  DSC
//
//  Created by Tuan Nguyen Trung on 6/16/15.
//  Copyright (c) 2015 Asger Nyman Christiansen. All rights reserved.
//

#include "sph_function.h"

void sph_function::deform(DSC2D::DeformableSimplicialComplex& dsc){
    auto init_time = std::chrono::system_clock::now();

    
    /*
     * Track the SPH interface
     */
    
    
    /*
     * Stretch the sph to reserve volume
     */
    
    
    /*
     * Update display
     */
    
    update_compute_time(init_time);
    init_time = std::chrono::system_clock::now();
    
    dsc.deform();
    
    update_deform_time(init_time);
}