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

    
    /*
     * Track the SPH interface
     */
    
    
    /*
     * Stretch the sph to reserve volume
     */
    
    
    /*
     * Update display
     */

    
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
                if (verts[v] == INVALID_IDX) {
                    verts[v] = v_idx++;
                }
            }
        }
    }
}
void sph_function::build_matrix(){
    
}