//
//  dynamics.h
//  DSC
//
//  Created by Tuan Nguyen Trung on 2/12/15.
//  Copyright (c) 2015 Asger Nyman Christiansen. All rights reserved.
//

#ifndef __DSC__dynamics__
#define __DSC__dynamics__

#include <stdio.h>
#include "define.h"
#include "DSC.h"
#include "texture_helper.h"
#include "curve.h"
#include "velocity_function.h"
#include "image.h"

struct dynamics_param{
    double alpha = 0.00; // Second derivative. Keep the curve short
    double beta = 0.0; // Forth derivative. Keep the curve straight
    double gamma = 20; // External force scale
    double mass = 50;
};

class dynamics : public DSC2D::VelocityFunc<> {
public:
    dynamics();
    ~dynamics();
    
    /**
     Returns the name of the velocity function.
     */
    virtual std::string get_name() const
    {
        return std::string("Image segmentation");
    }
    
    /**
     Computes the motion of each interface vertex and stores the destination in 
     the simplicial complex class.
     */
    virtual void deform(DSC2D::DeformableSimplicialComplex& dsc);
    
public:
    bool update_dsc(DSC2D::DeformableSimplicialComplex &dsc, texture_helper &tex);
    
#pragma mark - Debug
public:
    void draw_curve(DSC2D::DeformableSimplicialComplex &dsc);
    
#pragma mark - Data
private:
    std::vector<curve> curve_list_;
    dynamics_param d_param_;
    
#pragma mark - private
private:
    std::vector<curve> extract_curve(DSC2D::DeformableSimplicialComplex &dsc);
    void compute_internal_force(std::vector<curve> &curve_list,
                                DSC2D::DeformableSimplicialComplex &dsc);
    void compute_external_force(std::vector<curve> &curve_list
                                ,dsc_obj &complex
                                ,texture_helper &tex);
    void compute_displacement(dsc_obj &dsc);
    
private: // Utility
    curve::node_key get_next_key(curve & cu, std::vector<curve::node_key> pt_keys);
};

#endif /* defined(__DSC__dynamics__) */
