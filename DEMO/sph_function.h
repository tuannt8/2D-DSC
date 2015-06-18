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

class sph_function: public DSC2D::VelocityFunc<>{
public:
    sph *sph_mgr;
    DSC2D::DeformableSimplicialComplex *dsc_ptr;
    
private:
    arma::mat A;
    int nb_vert, nb_face; // No. of vertices and face inside
    HMesh::VertexAttributeVector<int> vert_idx;
    HMesh::FaceAttributeVector<int> face_idx;
    
    const int INVALID_IDX = -1;
private:
    void re_index_dsc();
    void build_matrix();
    
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
    virtual void deform(DSC2D::DeformableSimplicialComplex&);
    
    /**
     Returns wether the motion has finished.
     */
    virtual bool is_motion_finished(DSC2D::DeformableSimplicialComplex& dsc)
    {
        return false;
    }

private:
    DSC2D::vec2 get_area_derivative(DSC2D::vec2 P, DSC2D::vec2 A, DSC2D::vec2 B);
};

#endif /* defined(__DSC__sph_function__) */
