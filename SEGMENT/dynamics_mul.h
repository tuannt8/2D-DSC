//
//  dynamics_mul.h
//  DSC
//
//  Created by Tuan Nguyen Trung on 3/20/15.
//  Copyright (c) 2015 Asger Nyman Christiansen. All rights reserved.
//

#ifndef __DSC__dynamics_mul__
#define __DSC__dynamics_mul__

#include <stdio.h>
#include "define.h"
#include "image.h"

class dynamics_mul {
    
    
public:
    dynamics_mul();
    ~dynamics_mul();
    
    void  update_dsc(dsc_obj &dsc, image &img);
    
private:
    // temporary variable
    dsc_obj * s_dsc;
    image * s_img;
    
    // Mean intensity
    std::map<int, double> mean_inten_;
    
    std::map<int, double> alpha_map_;
private:
    void compute_mean_intensity(std::map<int, double> & mean_inten_o);
    void compute_intensity_force();
    void displace_dsc();
    void compute_curvature_force();
    
private:
    void displace_dsc_2();
    void debug_optimum_dt();
    void debug_optimum_dt_2();
    
    double furthest_move(Node_key nid, Vec2 direction);
    double energy_change(Node_key nid, Vec2 new_pos);
    
    double star_energy(Node_key nid, Vec2 new_pos);
    // intensity different in node link
    double intensity_energy(Node_key nid, Vec2 new_pos);
    // Interface length in node link
    double curve_length(Node_key nid, Vec2 new_pos);
};

#endif /* defined(__DSC__dynamics_mul__) */
