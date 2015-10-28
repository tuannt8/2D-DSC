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
#include "../Eigen/Sparse"

using namespace HMesh;
using namespace Eigen;

#define sign_(a) (a>0? 1:-1)


DSC2D::vec2 sph_function::Gravity(0,-GRAVITY_ABS);

void sph_function::deform(DSC2D::DeformableSimplicialComplex& dsc){
    dsc_ptr = &dsc;
    
    /*
     Displace DSC
     */
    // Gravity
    HMesh::VertexAttributeVector<DSC2D::vec2> vels(dsc_ptr->get_no_vertices(), DSC2D::vec2(0.0));
    for(auto vi = dsc.vertices_begin(); vi != dsc.vertices_end(); ++vi)
    {
        if (dsc.is_interface(*vi) or dsc.is_crossing(*vi))
        {
            vels[*vi] += Gravity;
        }
    }
    // curvature
    for (auto hekey : dsc_ptr->halfedges())
    {
        if (dsc_ptr->is_interface(hekey))
        {
            auto hew = dsc_ptr->walker(hekey);
            auto l = dsc_ptr->get_pos(hew.opp().vertex()) - dsc_ptr->get_pos(hew.vertex());
            l.normalize();
            vels[hew.vertex()] += l*1.1;
        }
    }
    
    
    for(auto vi = dsc.vertices_begin(); vi != dsc.vertices_end(); ++vi)
    {
        if(dsc.is_movable(*vi))
        {
            if (dsc.is_interface(*vi) or dsc.is_crossing(*vi))
            {
                auto ff = vels[*vi];
                dsc.set_destination(*vi, dsc.get_pos(*vi) + vels[*vi]);
            }
        }
    }
    
    dsc.deform();

    
    /*
     Volume lost compensation
     */
    double volumeLost = get_curent_volume() - m_V0;
    
    // Index the interface veritces
    HMesh::VertexAttributeVector<int> vIdxs(dsc_ptr->get_no_vertices(), INVALID_IDX);
    int num = 0;
    for (auto vkey : dsc_ptr->vertices())
    {
        if (dsc_ptr->is_interface(vkey) or dsc_ptr->is_crossing(vkey))
        {
            vIdxs[vkey] = num++;
        }
    }
    
    // Build the equation
    using EigMat = SparseMatrix<double>;
    using EigVec = VectorXd;
    
    EigMat M(1 + num, 2*num);
    for (auto ekey : dsc_ptr->halfedges())
    {
        if (dsc_ptr->is_interface(ekey))
        {
            auto hew = dsc_ptr->walker(ekey);
            if (dsc_ptr->get_label(hew.face()) != 1)
            {
                hew = hew.opp();
            }
            
            auto pt_tip = dsc_ptr->get_pos(hew.vertex());
            auto pt_root = dsc_ptr->get_pos(hew.opp().vertex());
            auto line = pt_tip - pt_root;
            DSC2D::vec2 norm = DSC2D::Util::normalize(DSC2D::vec2(line[1], -line[0]));
            double length = line.length();
            
            // Matrix M
            int idx1 = vIdxs[hew.vertex()];
            int idx2 = vIdxs[hew.opp().vertex()];
            M.coeffRef(0, 2*idx1) += norm[0]*length / 2.0;
            M.coeffRef(0, 2*idx1+1) += norm[1]*length / 2.0;
            
            M.coeffRef(0, 2*idx2) += norm[0]*length / 2.0;
            M.coeffRef(0, 2*idx2+1) += norm[1]*length / 2.0;
        }
    }
    
    EigVec B(1 + num);
    for (int i = 0; i < 1+num; i++)
    {
        B[i] = 0;
    }
    B[0] = -volumeLost;
    
    // Boundary
    int bound_count = 1;
    for (auto vkey : dsc_ptr->vertices())
    {
        if (dsc_ptr->is_interface(vkey) or dsc_ptr->is_crossing(vkey))
        {
            DSC2D::vec2 norm;
            if (!is_on_boundary(dsc_ptr->get_pos(vkey), norm))
            {
                auto n = dsc_ptr->get_normal(vkey);
                norm = DSC2D::vec2(n[1], -n[0]);
            }
            
            int idx = vIdxs[vkey];
            M.coeffRef(bound_count, 2*idx) = norm[0];
            M.coeffRef(bound_count, 2*idx+1) = norm[1];
            bound_count++;
        }
    }
    
    // Solve
    EigMat M_t = M.transpose();
    EigMat MtM = M_t*M;
    EigVec MtB = M_t*B;
    
    ConjugateGradient<EigMat> cg(MtM);
    EigVec X = cg.solve(MtB);
    
    // Apply displcement
    ver_dis.resize(dsc_ptr->get_no_vertices());
    for (auto vkey : dsc_ptr->vertices())
    {
        int idx = vIdxs[vkey];
        if (idx != INVALID_IDX)
        {
            DSC2D::vec2 dis(X[2*idx], X[2*idx+1]);
            dsc_ptr->set_destination(vkey, dsc_ptr->get_pos(vkey) + dis);
            ver_dis[vkey] = dis;
        }
    }
    
    dsc_ptr->deform();
    
    /*
     Project back to boundary
     */
    for (auto vkey : dsc_ptr->vertices())
    {
        if (dsc_ptr->is_interface(vkey) or dsc_ptr->is_crossing(vkey))
        {
            DSC2D::vec2 p_v;
            if (is_outside(dsc_ptr->get_pos(vkey), p_v))
            {
                dsc_ptr->set_destination(vkey, p_v);
            }
        }
    }
    dsc.deform();
}

bool sph_function::is_on_boundary(DSC2D::vec2 pt, DSC2D::vec2 &norm)
{
    double l = (pt - center_bound).length();
    if (l > r_bound - 1e-2)
    {
        norm = DSC2D::Util::normalize(pt-center_bound);
        return true;
    }
    
    return false;
}

bool sph_function::is_outside(DSC2D::vec2 pt, DSC2D::vec2 &projectionPoint)
{
    double l = (pt - center_bound).length();
    if (l > r_bound)
    {
        projectionPoint = center_bound + DSC2D::Util::normalize(pt-center_bound)*r_bound;
        return true;
    }
    
    return false;
}

double sph_function::get_curent_volume(){
    double V = 0;
    for (auto fkey: dsc_ptr->faces())
    {
        if (dsc_ptr->get_label(fkey) == 1)
        {
            V += dsc_ptr->area(fkey);
        }
    }
    
    return V;
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
    // Boundary
    int N = 150;
    glBegin(GL_LINE_LOOP);
    glColor3f(1, 0, 0);
    double step = 2*M_PI / (double)N;
    for (int i = 0; i < N; i++)
    {
        double aa = step*i;
        DSC2D::vec2 pt = center_bound + DSC2D::vec2(sin(aa)*r_bound, cos(aa)*r_bound);
        glVertex2d(pt[0], pt[1]);
    }
    
    glEnd();

    
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
    m_V0 = get_curent_volume();

    // Boundary
    auto pts = dsc_ptr->get_design_domain()->get_corners();
    
    center_bound = (pts[0] + pts[2]) / 2;
    r_bound = (pts[1] - pts[0]).length() * 0.5 * 0.8;
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

