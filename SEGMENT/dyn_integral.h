//
//  dyn_integral.h
//  DSC_seg_integral
//
//  Created by Tuan Nguyen Trung on 5/12/15.
//  Copyright (c) 2015 Asger Nyman Christiansen. All rights reserved.
//

#ifndef __DSC_seg_integral__dyn_integral__
#define __DSC_seg_integral__dyn_integral__

#include <stdio.h>
#include "define.h"
#include "image.h"

/*
 vertex attribute Vec2
 */
enum{
    FIRST_DERIVE = 0,
    SECOND_DEREIVE,
    FORCE,
};

/*
 vertex attribute double
 */
enum{
    E_MID = 0,
    E_X_0,
    E_X_1,
    E_Y_0,
    E_Y_1,
};

class dyn_integral{
private:
    dsc_obj * s_dsc;
    image * s_img;
    
    // Distance to compute derivative
    // Displacement of vertices should be around or smaller
    double epsilon_deriv = 6.0;
    
    std::map<int, double> mean_inten_;
    
public:
    dyn_integral(){};
    ~dyn_integral(){};
    
    
    void update_dsc(dsc_obj &dsc, image &img);
    
private:
    // Compute mean intensity
    void compute_mean_intensity(std::map<int, double> & mean_inten_o);
    
    // Compute energy if moving the vertex
    void compute_derivative();
    
    // Displace DSC
    void displace_dsc();
    
    
    // Compute energy in a star with asumption of new location of vertex
    bool energy_with_location_1(double &E, Node_key nkey , Vec2 displace, double * real_dis = nullptr);

    bool energy_with_location(double &E, Node_key nkey , Vec2 displace, double * real_dis = nullptr);
    void optimize_phase();
    double tri_energy_with_phase_assumtion(Face_key fkey, int assume_phase);
};

#endif /* defined(__DSC_seg_integral__dyn_integral__) */
