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
    void displace_dsc(dsc_obj *obj = nullptr);
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
    
private:
    double get_total_energy(dsc_obj *obj, std::map<int, double>  intesity_map);
    
    double energy_gradient_by_moving_distance(dsc_obj *obj, std::map<int, double>  intesity_map);
    double u_gradient(dsc_obj *obj, std::map<int, double>  intensity_map);
    double image_gradient_count(dsc_obj *obj, std::map<int, double>  intensity_map);
    double gradient_length(dsc_obj *obj);
    void get_curvature(dsc_obj *obj, HMesh::Walker hew, double &Kcur, double &Kpre);
    double get_curvature(dsc_obj *obj, HMesh::Walker hew);
    
    double optimal_dt(dsc_obj * clone);
};

#endif /* defined(__DSC__dynamics_mul__) */
