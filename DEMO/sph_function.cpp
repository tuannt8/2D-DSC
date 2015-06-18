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
    dsc_ptr = &dsc;
    
    /*
     * Index DSC
     */
    re_index_dsc();
    
    /*
     * Compute matrix A in: A dP = dV
     */
    build_matrix();
    
    /*
     * Compute expected volume
     */
    compt_volume_change();
    
    update_compute_time(init_time);
    init_time = std::chrono::system_clock::now();
    
    dsc.deform();
    
    update_deform_time(init_time);
}

void sph_function::re_index_dsc(){
    vert_idx = HMesh::VertexAttributeVector<int>(dsc_ptr->get_no_vertices(), INVALID_IDX);
    face_idx = HMesh::FaceAttributeVector<int>(dsc_ptr->get_no_faces(), INVALID_IDX);
    
    int f_idx = 0, v_idx = 0;
    for (auto fkey : dsc_ptr->faces()) {
        if (dsc_ptr->get_label(fkey) != 0) {
            
            face_idx[fkey] = f_idx++;
            
            auto verts = dsc_ptr->get_verts(fkey);
            for (auto v : verts) {
                if (vert_idx[v] == INVALID_IDX) {
                    vert_idx[v] = v_idx++;
                }
            }
        }
    }
    
    nb_vert = v_idx;
    nb_face = f_idx;
}

void sph_function::init(){
    double mass = sph_mgr->get_total_mass();
    V0 = 0.0;
    for (auto fkey:dsc_ptr->faces()){
        if (dsc_ptr->get_label(fkey) != 0) {
            V0 += dsc_ptr->area(fkey);
        }
    }
    
    rho_0 = mass / V0;
}

void sph_function::build_matrix(){
    A = arma::zeros(nb_face, nb_vert*2);
    for (auto fkey : dsc_ptr->faces()){
        if (dsc_ptr->get_label(fkey) != 0) {
            int fIdx = face_idx[fkey];
            
            auto verts = dsc_ptr->get_verts(fkey);
            for (int i = 0; i < 3; i++) {
                int vIdx = vert_idx[verts[i]];
                
                DSC2D::vec2 d = get_area_derivative(dsc_ptr->get_pos(verts[i]),
                                               dsc_ptr->get_pos(verts[(i+1)%3]),
                                               dsc_ptr->get_pos(verts[(i+2)%3]));
                
                A(fIdx, vIdx*2) = d[0];
                A(fIdx, vIdx*2 + 1) = d[1];
            }
        }
    }
}

void sph_function:: compt_volume_change(){
    // Now we approximate with 1 point only
    dV_col = arma::zeros(nb_face);
    
    
    for(auto fkey : dsc_ptr->faces()){
        if (dsc_ptr->get_label(fkey) != 0) {
            int fIdx = face_idx[fkey] ;
            
            auto v_pos = dsc_ptr->get_pos(fkey);
            auto center = (v_pos[0] + v_pos[1] + v_pos[2])/3;
            
            double rho = sph_mgr->get_intensity(center);
            
            double V = dsc_ptr->area(fkey);
            double dV = V*(rho/rho_0 - 1);
            
            dV_col(fIdx) = dV;
        }
    }
}

DSC2D::vec2 sph_function::get_area_derivative(DSC2D::vec2 P, DSC2D::vec2 A, DSC2D::vec2 B){
    double l = (B-A).length();
    
    // Project P to AB
    double t = CGLA::dot(P-A, B-A) / l;
    DSC2D::vec2 V = A + (B-A)*t;
    
    // Altitude of PAB
    DSC2D::vec2 n = P - V;
    
    n.normalize();
    return n*l;
}

