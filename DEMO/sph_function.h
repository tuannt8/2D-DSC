//
//  sph_function.h
//  DSC
//
//  Created by Tuan Nguyen Trung on 6/16/15.
//  Copyright (c) 2015 Asger Nyman Christiansen. All rights reserved.
//

#ifndef __DSC__sph_function__
#define __DSC__sph_function__

#include "velocity_function.h"
#include "sph.h"

class sph_function: public DSC2D::VelocityFunc<>{
public:
    sph *sph_mgr;
public:
    /**
     Creates a rotating velocity function.
     */
    sph_function(double velocity, double accuracy): VelocityFunc(M_PI*velocity/180., accuracy)
    {
        
    }
    
    /**
     Returns the name of the velocity function.
     */
    virtual std::string get_name() const
    {
        return std::string("SPH");
    }
    
    /**
     Computes the motion of each interface vertex and stores the destination in the simplicial complex class.
     */
    virtual void deform(DSC2D::DeformableSimplicialComplex& dsc);
    
    /**
     Returns wether the motion has finished.
     */
    virtual bool is_motion_finished(DSC2D::DeformableSimplicialComplex& dsc)
    {
        return false;
    }
};

#endif /* defined(__DSC__sph_function__) */
