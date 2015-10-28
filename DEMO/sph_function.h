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
    sph *sph_mgr;
    DSC2D::DeformableSimplicialComplex *dsc_ptr;
    
private:
    arma::mat A;
    arma::colvec dV_col;
    arma::colvec dP_col;
    HMesh::VertexAttributeVector<DSC2D::vec2> vert_displace;
    
    double rho_0, V0; // Constant intensity
    
    int nb_vert, nb_face; // No. of vertices and face inside
    HMesh::VertexAttributeVector<int> vert_idx;
    HMesh::FaceAttributeVector<int> face_idx;
    
private:
    // Constant volume
    double m_V0;
    
    // Boundary
    DSC2D::vec2 center_bound;
    double r_bound;
    
    
    //debug
public:
    HMesh::VertexAttributeVector<DSC2D::vec2> ver_dis;
    static DSC2D::vec2 Gravity;
private:
    double get_curent_volume();
    bool is_outside(DSC2D::vec2 pt, DSC2D::vec2 &projectionPoint);
    bool is_on_boundary(DSC2D::vec2 pt, DSC2D::vec2 &projectionPoint);
private:
    void re_index_dsc();
    void build_matrix();
    void compt_volume_change();
    void solve_displacement();
    void solve_displacement_constraint();
    void solve_displacement_tikhnov();
    void displace_sph();
    void displace_dsc();
public:
    void init();
    
    void draw();
    
    void get_info();
    /**
     * Fit DSC to SPH
     */
    void fit_dsc_to_sph();
    
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
private:
    DSC2D::vec2 get_area_derivative(DSC2D::vec2 P, DSC2D::vec2 A, DSC2D::vec2 B);
    double get_barrycentric_coord(DSC2D::vec2 p, DSC2D::vec2 P,
                                  DSC2D::vec2 B, DSC2D::vec2 C);
};

#endif /* defined(__DSC__sph_function__) */
