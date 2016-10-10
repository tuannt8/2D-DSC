
//
//  dynamics_prob.h
//  DSC_seg
//
//  Created by Tuan Nguyen Trung on 5/6/15.
//  Copyright (c) 2015 Asger Nyman Christiansen. All rights reserved.
//

#ifndef __DSC_seg__dynamics_prob__
#define __DSC_seg__dynamics_prob__

#include <stdio.h>
#include "define.h"
#include "image.h"

#define num_phase 2

class XI_f{
public:
    static double value(double s               // xác suất
                        , int phase                // Phase index
                        , int np = num_phase   // number of phase
                        );
    static double diff(double s               // xác suất
                       , int phase                // Phase index
                       , int np = num_phase   // number of phase);
    );
};
class K_Phi{
public:
    static double value(double s, int np = num_phase);
    static double diff(double s, int np = num_phase);
};

class dynamics_prob{
private:
    dsc_obj * dsc_;
    image * img_;
    
private:
//    arma::colvec C_vec; // Phase intensity
    double dt = 0.001;
public:
    dynamics_prob(){};
    ~dynamics_prob(){};
    
    void update_dsc(dsc_obj & obj, image & img);
    
private:
    void compute_c();
    void init_xac_suat();
    void update_tri_intensity();
    void compute_xac_suat_force();
    void update_xac_suat();
    void update_lambda();
    
private:
    double sign(double a){return (a>0)? 1 : -1;}
};

#endif /* defined(__DSC_seg__dynamics_prob__) */
