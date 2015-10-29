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
#include "armadillo"

#define INVALID_IDX -1
#define GRAVITY_ABS 3

class sph_function{
public:
    DSC2D::DeformableSimplicialComplex *dsc_ptr;
    
private:
    // Constant volume
    double m_V0;
    
    // Boundary
    DSC2D::vec2 center_bound;
    double r_bound;
    
public: //debug
    HMesh::VertexAttributeVector<DSC2D::vec2> ver_dis;
    static DSC2D::vec2 Gravity;
    
private:
    double get_curent_volume();
    bool is_outside(DSC2D::vec2 pt, DSC2D::vec2 &projectionPoint);
    bool is_on_boundary(DSC2D::vec2 pt, DSC2D::vec2 &projectionPoint);

public:
    void init();
    
    void draw();
    
    /**
     Returns the name of the velocity function.
     */
    std::string get_name() const
    {
        return std::string("SPH");
    }
    
    /**
     Computes the motion of each interface vertex and stores the destination in the simplicial complex class.
     */
    void deform(DSC2D::DeformableSimplicialComplex&);
    

    sph_function() {};
};

#endif /* defined(__DSC__sph_function__) */
