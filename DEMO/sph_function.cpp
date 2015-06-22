//
//  sph_function.cpp
//  DSC
//
//  Created by Tuan Nguyen Trung on 6/16/15.
//  Copyright (c) 2015 Asger Nyman Christiansen. All rights reserved.
//

#include "sph_function.h"
#include "draw.h"
#include "console_debug.h"


void sph_function::deform(DSC2D::DeformableSimplicialComplex& dsc){
    
    /*
     * Displace SPH
     */
    displace_sph();
    
    /*
     * Deform DSC
     */
    displace_dsc();
    
    /*
     * Index DSC
     */
    re_index_dsc();
    
    /*
     * Compute matrix A in: A dP_col = dV_col
     */
    build_matrix();
    
    /*
     * Compute expected volume dV_col
     */
    compt_volume_change();
    
    /**
     * Solve for displacement dP_col
     */
//    solve_displacement();
    solve_displacement_tikhnov();

    get_info();
}

void sph_function::fit_dsc_to_sph(){
    vert_displace = HMesh::VertexAttributeVector<DSC2D::vec2>(dsc_ptr->get_no_vertices(), DSC2D::vec2(0.0));

    for (auto ekey : dsc_ptr->halfedges()) {
        auto hew = dsc_ptr->walker(ekey);
        
        if (HMesh::boundary(*dsc_ptr->mesh, ekey)) {
            continue;
        }
        
        if (dsc_ptr->get_label(hew.face()) == 1
            and dsc_ptr->get_label(hew.opp().face()) == 0)
        {
            auto p1 = dsc_ptr->get_pos(hew.vertex());
            auto p2 = dsc_ptr->get_pos(hew.opp().vertex());
            
            int num_point = (int)(p2 - p1).length();
            double dl = (p2 - p1).length() / (double)(num_point - 1);
            
            double dt = 1000;
            
            double f1 = 0.0, f2 = 0.0;
            auto p12 = p2 - p1;
            p12.normalize();
            for (int i = 0; i < num_point; i++) {
                auto p = p1 + p12*(i+0.5)*dl;
                double rho = sph_mgr->get_intensity(p);
                double f = (2*rho - rho_0)*dt;
                f1 += f * (p - p2).length() / (p2-p1).length();
                f2 += f * (p - p1).length() / (p2 - p1).length();
            }
            
            DSC2D::vec2 N = (p1 - p2);
            DSC2D::vec2 outer_N(N[1], -N[0]);
            outer_N.normalize();
            
            vert_displace[hew.vertex()] += outer_N*f1;
            vert_displace[hew.opp().vertex()] += outer_N*f2;
        }
    }
    
    // Displace
    for (auto nkey : dsc_ptr->vertices()){
        if (dsc_ptr->is_interface(nkey)) {
            dsc_ptr->set_destination(nkey, dsc_ptr->get_pos(nkey) + vert_displace[nkey]);
        }
    }
    
    dsc_ptr->deform();
    
    get_info();
}

void sph_function::displace_dsc(){
    if (vert_displace.size() == 0) {
        return;
    }
    
    for (auto vkey : dsc_ptr->vertices()) {
        dsc_ptr->set_destination(vkey,
                    dsc_ptr->get_pos(vkey) + vert_displace[vkey]);
    }
    dsc_ptr->deform();
}

void sph_function::displace_sph()
{
    if (vert_displace.size() == 0) {
        return;
    }
    
    for (int i = 0; i < sph_mgr->point_.size(); i++)
    {
        // Find the triangle that contains the SPH; Optimize later
        auto pt = sph_mgr->point_[i];
        for (auto fkey : dsc_ptr->faces())
        {
            auto tris = dsc_ptr->get_pos(fkey);
            if (DSC2D::Util::is_inside(pt, tris))
            {
                // Displace the point linearly
                DSC2D::vec2 dis(0.0);
                auto verts = dsc_ptr->get_verts(fkey);
                for (int j = 0; j < 3; j++)
                {
                    double barryC = get_barrycentric_coord(pt, tris[j],
                                            tris[(j+1)%3], tris[(j+2)%3]);
                    assert(barryC >= 0);
                    dis += vert_displace[verts[j]]*barryC;
                }
               
                double bc[3];
                if(fkey.get_index() == 176)
                {
   
                    bc[0] = get_barrycentric_coord(pt, tris[0], tris[1], tris[2]);
                    bc[1] = get_barrycentric_coord(pt, tris[1], tris[0], tris[2]);
                    bc[2] = get_barrycentric_coord(pt, tris[2], tris[0], tris[1]);
                }
                
                sph_mgr->point_[i] = pt + dis;
                break;
            }
        }
    }
}

double sph_function::get_barrycentric_coord(DSC2D::vec2 p, DSC2D::vec2 P,
                              DSC2D::vec2 B, DSC2D::vec2 C)
{
    return DSC2D::Util::area(p, B, C) / DSC2D::Util::area(P, B, C);
}

void sph_function::solve_displacement_constraint(){
    
    /*
     * Add boundary contrain
     */
    std::vector<DSC2D::DeformableSimplicialComplex::node_key> bound_nodes;
    DSC2D::vec2 up(0,1);
    for (auto nkey:dsc_ptr->vertices())
    {
        if (dsc_ptr->is_interface(nkey))
        {
            auto norm = dsc_ptr->get_normal(nkey);
            if (DSC2D::Util::dot(up, norm) < 0.7 ) {
                bound_nodes.push_back(nkey);
            }
        }
    }
    arma::mat A_bound = arma::zeros(nb_face + bound_nodes.size()*2, nb_vert*2);
    A_bound.submat(0, 0, nb_face-1, nb_vert*2-1) = A;
    
    double LARGE = 1e5;
    
    int r_idx = nb_face;
    for (auto nkey: bound_nodes) {
        // Fix
        int idx = vert_idx[nkey];
        A_bound(r_idx++, idx*2) = LARGE;
        A_bound(r_idx++, idx*2+1) = LARGE;
        
    }
    
    arma::colvec dV_bound(nb_face + bound_nodes.size()*2);
    dV_bound.submat(0, 0, nb_face-1, 0) = dV_col;
    
    /*
     * Sovle it
     */
    arma::mat A_t = A_bound.t();
    arma::mat AtA = A_t * A_bound;
    
    A.save("A.txt", arma::raw_ascii);
    A_bound.save("A_bound.txt", arma::raw_ascii);
    
    if(arma::det(AtA) < 0.00001){
        printf("Matrix singular");
    }
    
    dP_col = arma::solve(AtA, A_t*dV_bound);
    vert_displace = HMesh::VertexAttributeVector<DSC2D::vec2>
    (dsc_ptr->get_no_vertices(), DSC2D::vec2(0.0));
    double max_dis = 0;
    double dt = 0.02;
    for (auto nkey : dsc_ptr->vertices()) {
        if (vert_idx[nkey] != INVALID_IDX) {
            int idx = vert_idx[nkey];
            vert_displace[nkey] = DSC2D::vec2(dP_col[idx*2], dP_col[idx*2 + 1]) * dt;
            
            if (max_dis < vert_displace[nkey].length()) {
                max_dis = vert_displace[nkey].length();
            }
        }
    }
}

void sph_function::solve_displacement_tikhnov(){
    arma::mat A_t = A.t();
    arma::mat AtA = A_t*A;
    
    // Tikhonov regularization
    double alpha = 0.01;
    arma::mat G = arma::zeros(AtA.n_rows, AtA.n_cols);
    G.eye();
    G = G*alpha;
    auto A_tik = AtA + G.t()*G;
    
    if(arma::det(A_tik) < 0.00001){
        printf("Matrix singular");
    }
    
    dP_col = arma::solve(A_tik, A_t*dV_col);
    
    vert_displace = HMesh::VertexAttributeVector<DSC2D::vec2>
    (dsc_ptr->get_no_vertices(), DSC2D::vec2(0.0));
    double max_dis = 0;
    
    double dt = 0.07;
    
    for (auto nkey : dsc_ptr->vertices()) {
        if (vert_idx[nkey] != INVALID_IDX) {
            int idx = vert_idx[nkey];
            vert_displace[nkey] = DSC2D::vec2(dP_col[idx*2], dP_col[idx*2 + 1]) * dt;
            
            if (max_dis < vert_displace[nkey].length()) {
                max_dis = vert_displace[nkey].length();
            }
        }
    }
}

void sph_function::solve_displacement(){

    arma::mat A_t = A.t();
    arma::mat AtA = A_t*A;
    
    if(arma::det(AtA) < 0.00001){
        printf("Matrix singular");
    }
    
    A.save("A.txt", arma::raw_ascii);

    dP_col = arma::solve(AtA, A_t*dV_col);
    
    vert_displace = HMesh::VertexAttributeVector<DSC2D::vec2>
                        (dsc_ptr->get_no_vertices(), DSC2D::vec2(0.0));
    double max_dis = 0;
    double dt = 0.02;
    for (auto nkey : dsc_ptr->vertices()) {
        if (vert_idx[nkey] != INVALID_IDX) {
            int idx = vert_idx[nkey];
            vert_displace[nkey] = DSC2D::vec2(dP_col[idx*2], dP_col[idx*2 + 1]) * dt;
            
            if (max_dis < vert_displace[nkey].length()) {
                max_dis = vert_displace[nkey].length();
            }
        }
    }
}

void sph_function::draw(){
    // Face intensity
    if (console_debug::get_opt("DSC Face color by SPH", true)) {
        HMesh::FaceAttributeVector<DSC2D::vec3>
            colors(dsc_ptr->get_no_faces(), DSC2D::vec3(0.0));
        
        double scale = 10000;
//        double rr = rho_0 * scale;
        
//        printf("\n ============================= \n");
        for (auto fkey : dsc_ptr->faces()){
//            if (dsc_ptr->get_label(fkey) != 0)
            {
                auto v_pos = dsc_ptr->get_pos(fkey);
                double mass = sph_mgr->get_mass_tri(v_pos);
                double rho = mass /dsc_ptr->area(fkey)*scale;
                colors[fkey] = DSC2D::vec3(rho, rho, rho);
//                printf(" %f ", rho/scale);
            }
//            else{
//                colors[fkey] = DSC2D::vec3(rr, rr, rr);
//            }

        }
        Painter::draw_faces(*dsc_ptr, colors);
    }
    
    // Vetex displacement
    if (console_debug::get_opt("Vert displacement", true)) {
        if (vert_displace.size() == dsc_ptr->get_no_vertices()) {
            Painter::draw_arrows(*dsc_ptr, vert_displace);
        }
    }
    
    // Volume change
    if (console_debug::get_opt("Volume change", true)) {
        if (dP_col.n_elem > 0) {
            HMesh::FaceAttributeVector<std::string> vc(dsc_ptr->get_no_faces(), "");
            for (auto fkey:dsc_ptr->faces()) {
                if (face_idx[fkey] != INVALID_IDX) {
                    double dV = dV_col[face_idx[fkey]];
                    vc[fkey] = dV>0? "+" : "-";
                }
            }
            Painter::draw_faces_text(*dsc_ptr, vc);
        }
    }
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
    
    
    // No of face and index inside
    nb_vert = v_idx;
    nb_face = f_idx;
}

void sph_function::init(){
    double mass = 0.0;
    V0 = 0.0;
    for (auto fkey:dsc_ptr->faces()){
        if (dsc_ptr->get_label(fkey) != 0) {
            
            V0 += dsc_ptr->area(fkey);
            auto v_pos = dsc_ptr->get_pos(fkey);
            mass += sph_mgr->get_mass_tri(v_pos);
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

void sph_function::get_info(){
    double mass = 0.0;
    double V = 0.0;
    for (auto fkey:dsc_ptr->faces()){
        if (dsc_ptr->get_label(fkey) != 0) {
            
            V += dsc_ptr->area(fkey);
            auto v_pos = dsc_ptr->get_pos(fkey);
            mass += sph_mgr->get_mass_tri(v_pos);
        }
    }
    
    double rh = mass / V;
    printf("Mass: %f; V: %f; density: %f; default density: %f\n", mass, V, rh, rho_0);
}

void sph_function:: compt_volume_change(){
    // Now we approximate with 1 point only
    dV_col = arma::zeros(nb_face);
    
    for(auto fkey : dsc_ptr->faces()){
        if (dsc_ptr->get_label(fkey) != 0) {
            int fIdx = face_idx[fkey] ;
            
            auto v_pos = dsc_ptr->get_pos(fkey);
            double mass = sph_mgr->get_mass_tri(v_pos);
            
            double V = dsc_ptr->area(fkey);
            double dV = mass / rho_0 - V;
            
            dV_col(fIdx) = dV;
        }
    }
}

DSC2D::vec2 sph_function::get_area_derivative(DSC2D::vec2 P, DSC2D::vec2 A, DSC2D::vec2 B){
    double l = (B-A).length();
    
    // Project P to AB
    double t = CGLA::dot(P-A, B-A) / l / l;
    DSC2D::vec2 V = A + (B-A)*t;
    
    // Altitude of PAB
    DSC2D::vec2 n = P - V;
    
    n.normalize();
    return n*l;
}

