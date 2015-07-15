//
//  dynamics_mul.cpp
//  DSC
//
//  Created by Tuan Nguyen Trung on 3/20/15.
//  Copyright (c) 2015 Asger Nyman Christiansen. All rights reserved.
//

#include "dynamics_mul.h"
#include "helper.h"
#ifdef WIN32
#include <CGLA/Mat3x3f.h>
#else
#include <GEL/CGLA/Mat3x3f.h>
#endif

#include <armadillo>
#include "define.h"
#include "util.h"
#include "polygon_f.h"

using namespace std;

#define GET_FACE_IDX(fid) (int)s_dsc->get_phase_attr(fid, FACE_IDX][0]

dynamics_mul::dynamics_mul(){
    
}

dynamics_mul::~dynamics_mul(){
    
}

void dynamics_mul::update_dsc_explicit(dsc_obj &dsc, image &img){
    
    s_img = &img;
    s_dsc = &dsc;
    
    // 1. Update mean intensity
    // <phase - mean intensity>
//    compute_mean_intensity(mean_inten_);
    
    // 4. Update DSC
//    E0_ = get_total_energy(s_dsc, mean_inten_);
//    
//    static int iter = 0;
//    printf("%d %f \n", iter++, E0_);
    
//    displace_dsc();
    
//    E1_ = get_total_energy(s_dsc, mean_inten_);
//
//    if (dE_0_ < 0.) {
//        double ee = E0_ - E1_;
//        double gap = -1./2.*dE_0_/(E1_ - E0_ - dE_0_);
//        
//        if(gap < 0){
//            // The prediction is so incorrect
//            // Energy reduce more than expected
//            
//        }
//        else if(gap < 1)
//            dt = dt*gap;
//
//        printf("dt = %f \n", dt);
//    }
    
 //   optimize_phase();
    
    // 4. Update DSC
    displace_dsc();
    
    compute_mean_intensity(mean_inten_);
    
  //  compute_mean_intensity(mean_inten_);
    g_param.mean_intensity = mean_inten_; // For drawing
    
    // 2. Compute intensity force
    //      External force attributes
    compute_intensity_force();
    
    // 3. Curvature force
    compute_curvature_force();
    
//    compute_difference();
}
void dynamics_mul::compute_difference()
{
    for (auto nkey : s_dsc->vertices())
    {
        double total = 0.0;
        int count_v = 0;
        for(auto hew = s_dsc->walker(nkey); !hew.full_circle(); hew = hew.circulate_vertex_cw())
        {
            auto fid = hew.face();
            
            if (fid == HMesh::InvalidFaceID) {
                continue;
            }
            
            double ci = mean_inten_[s_dsc->get_label(fid)];
            auto tris = s_dsc->get_pos(fid);
            
            Vec2 min(INFINITY, INFINITY), max(-INFINITY, -INFINITY);
            for (auto p: tris){
                min[0] = std::min(min[0], p[0]);
                min[1] = std::min(min[1], p[1]);
                max[0] = std::max(max[0], p[0]);
                max[1] = std::max(max[1], p[1]);
            }
            
            double tri_total = 0.0;
            int count = 0;
            for (int i = floor(min[0]); i < ceil(max[0]); i++) {
                for (int j = floor(min[1]); j < ceil(max[1]); j++) {
                    if (helper_t::is_point_in_tri(Vec2(i,j), tris)) {
                        double I = s_img->get_intensity(i, j);
                        tri_total += (I-ci)*(I-ci);
                        count ++;
                    }
                }
            }
            
            count_v += count;
            total += tri_total;
        } /* for(auto hew) */
        
        s_dsc->set_node_force(nkey, Vec2(total/(double)count_v), STAR_DIFFER);
    } /* for (auto nkey) */
}

void dynamics_mul::update_dsc(dsc_obj &dsc, image &img){
    // update_dsc_implicit(dsc, img);
     update_dsc_explicit(dsc, img);
    // update_dsc_explicit_whole_domain(dsc, img);
    // update_dsc_area(dsc, img);
    // update_dsc_build_and_solve(dsc, img);
}

void dynamics_mul::update_dsc_build_and_solve(dsc_obj &dsc, image &img){
    s_img = &img;
    s_dsc = &dsc;
    
    s_dsc->deform();
    
    // 1. Update mean intensity
    // map<phase - mean intensity>
    compute_mean_intensity(mean_inten_);
    g_param.mean_intensity = mean_inten_; // For drawing
    
    indexing_vertices();
    
    int no_faces = s_dsc->get_no_faces();
    int no_p = s_dsc->get_no_vertices();
    
    // Add boundary condition
    int nb_bound = 0;
    for (auto nkey : s_dsc->vertices()) {
        if (HMesh::boundary(*s_dsc->mesh, nkey)) {
            nb_bound ++;
        }
    }
    
    arma::colvec det_col(no_faces); det_col.fill(0.);
    arma::mat A_mat(no_faces, no_p*2); A_mat.fill(0.);
    

    
    /*
     de/dJ
     */
    double dt = 1000.00;
    for (auto fid = s_dsc->faces_begin(); fid != s_dsc->faces_end(); fid++) {
        auto node_idxs = s_dsc->get_verts(*fid); // key of node
        
        
        std::vector<Vec2> tris = s_dsc->get_pos(*fid);
        double area = s_dsc->area(*fid);
        double ci = mean_inten_[s_dsc->get_label(*fid)];
        
        Vec2 min(INFINITY, INFINITY), max(-INFINITY, -INFINITY);
        for (auto p: tris){
            min[0] = std::min(min[0], p[0]);
            min[1] = std::min(min[1], p[1]);
            max[0] = std::max(max[0], p[0]);
            max[1] = std::max(max[1], p[1]);
        }
        
        double differ = 0.;
        for (int i = floor(min[0]); i < ceil(max[0]); i++) {
            for (int j = floor(min[1]); j < ceil(max[1]); j++) {
                if (helper_t::is_point_in_tri(Vec2(i,j), tris)) {
                    differ += std::pow(( (s_img->get_intensity(i, j)) - ci), 2);
                }
            }
        }
        
        differ /= area;
        
        int fIdx = (int)s_dsc->get_phase_attr(*fid, FACE_IDX)[0];
        
        det_col(fIdx) = - differ * dt;
    }
    
    /*
     Matrix A
     */
    for (auto fid = s_dsc->faces_begin(); fid != s_dsc->faces_end(); fid++) {
        auto node_keys = s_dsc->get_verts(*fid);
        int fIdx = (int)s_dsc->get_phase_attr(*fid, FACE_IDX)[0];
        
        for (int i = 0; i < 3; i++) {
            auto A_key = node_keys[i];
            auto B_key = node_keys[(i+1)%3];
            auto C_key = node_keys[(i+2)%3];
            
            auto A = s_dsc->get_pos(A_key);
            auto B = s_dsc->get_pos(B_key);
            auto C = s_dsc->get_pos(C_key);
            
            
            double t = DSC2D::Util::dot(A-B, C-B) / ((C-B).length() * (C-B).length());
            Vec2 H = B + (C-B) * t;
            Vec2 HA = A - H;
            HA.normalize();
            
            Vec2 dJBC = (C-B).length() * HA;
            
            int A_idx = (int)s_dsc->get_node_force(A_key, INDEX_VERT)[0];
            
            A_mat(fIdx, 2*A_idx) = dJBC[0];
            A_mat(fIdx, 2*A_idx + 1) = dJBC[1];
        }
    }
    
    arma::mat real_A_mat = A_mat.submat(0, 0, no_faces-1, (no_p - nb_bound)*2 - 1);
    A_mat = real_A_mat;
    /*
     Solve A_mat * dP = det_col
     */
    arma::mat A_t = A_mat.t();
    arma::mat AtA = A_t*A_mat;
    arma::colvec AtdE = A_t * det_col;
    
    det_col.save(string(DATA_PATH) + "de.txt", arma::raw_ascii);
    real_A_mat.save(string(DATA_PATH) + "A.txt", arma::raw_ascii);
    
    
    arma::colvec dP = arma::solve( AtA, AtdE);
    
    
    // Assign displacement
    double max_dis = 0;
    for (auto nkey = s_dsc->vertices_begin(); nkey != s_dsc->vertices_end(); nkey++) {
        
        int n_idx = s_dsc->get_node_force(*nkey, INDEX_VERT)[0];
        
        if (n_idx >= dP.n_elem/2) {
            continue;
        }
        
        Vec2 f(dP(n_idx*2), dP(n_idx*2+1));
        
        if (s_dsc->is_interface(*nkey) or s_dsc->is_crossing(*nkey)) {
            

            
            s_dsc->set_destination(*nkey, s_dsc->get_pos(*nkey) + f);
            
            s_dsc->set_node_external_force(*nkey, f * 10);
            
            if (max_dis < f.length()) {
                max_dis = f.length();
            }
        }
        else
            s_dsc->set_node_internal_force(*nkey, f*10);
    }
    
    std::cout << "Max move; " << max_dis << std::endl;
}

void dynamics_mul::update_dsc_area(dsc_obj &dsc, image &img){
    s_img = &img;
    s_dsc = &dsc;
    
    s_dsc->deform();
    
    // 1. Update mean intensity
    // map<phase - mean intensity>
    compute_mean_intensity(mean_inten_);
    g_param.mean_intensity = mean_inten_; // For drawing
    
    //
    for (auto fid = s_dsc->faces_begin(); fid != s_dsc->faces_end(); fid++) {
        auto node_idxs = s_dsc->get_verts(*fid);
        std::vector<Vec2> tris = s_dsc->get_pos(*fid);
        double area = s_dsc->area(*fid);
        double ci = mean_inten_[s_dsc->get_label(*fid)];
        
        Vec2 min(INFINITY, INFINITY), max(-INFINITY, -INFINITY);
        for (auto p: tris){
            min[0] = std::min(min[0], p[0]);
            min[1] = std::min(min[1], p[1]);
            max[0] = std::max(max[0], p[0]);
            max[1] = std::max(max[1], p[1]);
        }
        
        // We have to compute further
        double signed_a = DSC2D::Util::signed_area(tris[0], tris[1], tris[2]) / area;
        double distance = 10.0; // Belong on how large epsilon is.
        double epsilon = 1.0;
        
        std::vector<Vec2> p_f(3, Vec2(0.));
        for (int i = floor(min[0]); i < ceil(max[0]); i++) {
            for (int j = floor(min[1]); j < ceil(max[1]); j++) {
                if (helper_t::distance_to_edge(Vec2(i,j), tris) < distance) {
                    
                    Vec2 p_c((double)i,(double)j);
                    double area[3];
                    area[0] = DSC2D::Util::signed_area(p_c, tris[1], tris[2])/signed_a;
                    area[1] = DSC2D::Util::signed_area(tris[0], p_c, tris[2])/signed_a;
                    area[2] = DSC2D::Util::signed_area(tris[0], tris[1], p_c)/signed_a;
                    
                    for (int k = 0; k < 3; k++) {
                        // Compute force
                        
                    }
                }
            }
        }
    }
}

HMesh::Walker dynamics_mul::pre_edge(dsc_obj *obj, HMesh::Walker hew){
    auto pre_e = hew.circulate_vertex_ccw();
    for (; !pre_e.full_circle(); pre_e = pre_e.circulate_vertex_ccw()) {
        if (obj->is_interface(pre_e.halfedge())) {
            pre_e = pre_e.opp();
            break;
        }
    }
    assert(obj->is_interface(pre_e.halfedge()));
    assert( pre_e.vertex() == hew.opp().vertex());
    return pre_e;
}

HMesh::Walker dynamics_mul::next_edge(dsc_obj *obj, HMesh::Walker hew){
    auto next_e = hew.opp().circulate_vertex_cw();
    for (; !next_e.full_circle(); next_e = next_e.circulate_vertex_cw()) {
        if (obj->is_interface(next_e.halfedge())) {            break;
        }
    }
    assert(obj->is_interface(next_e.halfedge()));
    assert( next_e.opp().vertex() == hew.vertex());
    
    return next_e;
}

Vec2 dynamics_mul::get_vertex_norm(dsc_obj *obj, HMesh::Walker hew){
    auto p = obj->get_pos(hew.vertex());
    auto p_opp = obj->get_pos(hew.opp().vertex());
    
    Vec2 N;
    auto next_e = next_edge(obj, hew);
    
    Vec2 Ne = p - p_opp;
    Ne.normalize();
    Ne = Vec2(Ne[1], -Ne[0]);
    
    Vec2 Ne_next = obj->get_pos(next_e.vertex()) - p;
    Ne_next.normalize();
    Ne_next = Vec2(Ne_next[1], -Ne_next[0]);
    
    N = Ne + Ne_next;
    N.normalize();
    return N;
}

void dynamics_mul::update_dsc_explicit_whole_domain(dsc_obj &dsc, image &img){
    s_img = &img;
    s_dsc = &dsc;
    
    E0_ = get_total_energy(s_dsc, mean_inten_);
    
    s_dsc->deform();
    


    
    // 1. Update mean intensity
    // map<phase - mean intensity>
    compute_mean_intensity(mean_inten_);
    g_param.mean_intensity = mean_inten_; // For drawing
    
    E1_ = get_total_energy(s_dsc, mean_inten_);
    
    double delta = 0.;
    if (dE2 > 0) {
        dE2 = - std::abs(dE2);
        double E01 = (E1_ - E0_);
        delta = (double)-(dE2) / (E01 - dE2) /2.;
        dt = dt*delta;
    }
    printf("%f %f %f %f %f %f\n", E1_, E0_, E1_ - E0_, dE2, delta, dt);
    
    // 5. Build and solve matrix
    
    int width = s_img->width();
    int height = s_img->height();
    
    /*
     1. Image gradient force
     */
    double max_grad = 0.0;
    for (auto fid = s_dsc->faces_begin(); fid != s_dsc->faces_end(); fid++) {
        auto node_idxs = s_dsc->get_verts(*fid);
        bool bInterface = false;
        for (auto n: node_idxs) {
            if (s_dsc->is_interface(n) || s_dsc->is_crossing(n)) {
                bInterface = true;
            }
        }
        if (!bInterface) {
            continue;
        }
        
        auto tris = s_dsc->get_pos(*fid);
        std::vector<int> idxs = get_vert_idx(node_idxs);
        
        double ci = mean_inten_[s_dsc->get_label(*fid)];
        
        Vec2 min(INFINITY, INFINITY), max(-INFINITY, -INFINITY);
        for (auto p: tris){
            min[0] = std::min(min[0], p[0]);
            min[1] = std::min(min[1], p[1]);
            max[0] = std::max(max[0], p[0]);
            max[1] = std::max(max[1], p[1]);
        }
        

        
        double area = s_dsc->area(*fid);
        
        for (int i = floor(min[0]); i < ceil(max[0]); i++) {
            for (int j = floor(min[1]); j < ceil(max[1]); j++) {
                if (helper_t::is_point_in_tri(Vec2(i,j), tris)) {
                    
                    // FORCE
                    Vec2 f = s_img->grad(i, j) * 2 * (s_img->get_intensity(i, j) - ci);
                    
                    
                    double l = f.length();
                    if (max_grad < l) {
                        max_grad = l;
                    }
                    // Barry centric linear interpolation
                    Vec2 pos = Vec2(i,j) + Vec2(0.5, 0.5);
                    Vec3 barray_coord;
                    for (int k = 0; k < 3; k++) {
                        barray_coord[k] = DSC2D::Util::signed_area(pos,
                                                                   tris[(k+1)%3],
                                                                   tris[(k+2)%3])
                        / DSC2D::Util::signed_area(tris[0],
                                                   tris[1],
                                                   tris[2]);
                        
                        // Set the force here
                        s_dsc->add_node_internal_force(
                                        node_idxs[k], f*barray_coord[k]
                        );
                        
                        double gg = f.length()*f.length()
                                    *barray_coord[k]*barray_coord[k];
                //        s_dsc->add_node_force(node_idxs[k], Vec2(gg, gg), E2_GRAD);
                        
                        s_dsc->add_node_force(node_idxs[k], Vec2(1.0,0.0), V_COUNT);
                        
                    } /* For three points */
                    
                } /* If inside */
            } /* For all point in face */
        } /* For all point in face */
    } /* For all faces */

    
    /*
     2. Boundary intensity forces
     */
    double max_inten = 0.0;
    
//    for (auto eit = s_dsc->halfedges_begin(); eit != s_dsc->halfedges_end(); eit++) {
//        if(s_dsc->is_interface(*eit)){
//            auto hew = s_dsc->walker(*eit);
//            
//            double c = mean_inten_[s_dsc->get_label(hew.face())];
//            double c_opp = mean_inten_[s_dsc->get_label(hew.opp().face())];
//            
//            // Loop on the edge
//            Vec2 p_opp = s_dsc->get_pos(hew.opp().vertex());
//            Vec2 p = s_dsc->get_pos(hew.vertex());
//            
//            int length = (int)(p - p_opp).length();
//            
////            double area = s_dsc->area(hew.face());
////            double area_opp = s_dsc->area(hew.opp().face());
//            
//            Vec2 norm = get_vertex_norm(s_dsc, hew);
//            Vec2 norm_pre = get_vertex_norm(s_dsc, pre_edge(s_dsc, hew));
//            
//            Vec2 L01 = p - p_opp;
//            L01.normalize();
//            Vec2 N01(L01[1], -L01[0]);
//            
//            
//            for (int i = 0; i <= length; i++) {
//                Vec2 p_cur = p_opp + (p - p_opp)*(double(i)/(double)length);
//                double I = s_img->get_intensity(p_cur[0], p_cur[1]);
//                
//                double psi = i / (double)length;
//                double psi_opp = (length - i) / (double)length;
//                
//                Vec2 N_cur = psi * norm + psi_opp * norm_pre;
//                N_cur.normalize();
//            //    N_cur = N01;
//                // Normalize force
//                Vec2 f = N_cur * (c - c_opp) * (4*I - (3*c + c_opp)) / 4.;
//                
//                
//                double l = f.length();
//                if (max_inten < l) {
//                    max_inten = l;
//                }
//                
//                s_dsc->add_node_external_force(hew.vertex(), f*psi);
//                s_dsc->add_node_external_force(hew.opp().vertex(), f*psi_opp );
//                
//            }
//        }
//    }
    
    HMesh::HalfEdgeAttributeVector<int> touched(s_dsc->get_no_halfedges(), 0);
    for (auto eit = s_dsc->halfedges_begin(); eit != s_dsc->halfedges_end(); eit++) {
        if(s_dsc->is_interface(*eit) and !touched[*eit]){
            auto hew = s_dsc->walker(*eit);
            
            double c0 = mean_inten_[s_dsc->get_label(hew.face())];
            double c1 = mean_inten_[s_dsc->get_label(hew.opp().face())];
            
            // Loop on the edge
            auto p0 = s_dsc->get_pos(hew.opp().vertex());
            auto p1 = s_dsc->get_pos(hew.vertex());
            
            int length = (int)(p1 - p0).length();
            
            double area0 = s_dsc->area(hew.face());
            double area1 = s_dsc->area(hew.opp().face());
            double avg = (area0 + area1)/2.;
            
            
            Vec2 L01 = p1 - p0;
            L01.normalize();
            Vec2 N01(L01[1], -L01[0]);
            
            for (int i = 0; i <= length; i++) {
                auto p = p0 + (p1 - p0)*(double(i)/(double)length);
                double I = s_img->get_intensity(p[0], p[1]);

                
                // Normalize force
                Vec2 f = N01 * ( (c0-c1)*(2*I - c0 - c1));// / ((c0-c1)*(c0-c1));
                
                
                double l = f.length();
                if (max_inten < l) {
                    max_inten = l;
                }

                double psi_opp = i/(double)length; // (p - p1).length() / (double)length
                double psi = (length - i) / (double)length; // (p - p0).length() / (double)length
                
                s_dsc->add_node_external_force(hew.opp().vertex(), f* psi);
                s_dsc->add_node_external_force(hew.vertex(), f* psi_opp);
                
                s_dsc->add_node_force(hew.opp().vertex(), Vec2(0,1), V_COUNT);
                s_dsc->add_node_force(hew.vertex(), Vec2(0,1), V_COUNT);
                
                double epsilon = 1.;
                double ff0 = (double)f.length()*(double)f.length()*
                        std::pow( (p - p1).length() / length , 2) / 2. / epsilon;
                double ff1 = f.length()*f.length()*std::pow( (p - p0).length() / length , 2) / 2. / epsilon;
                
                s_dsc->add_node_force(hew.opp().vertex(), Vec2(ff0, ff0), E2_GRAD);
                s_dsc->add_node_force(hew.vertex(), Vec2(ff1, ff1), E2_GRAD);
                
            }
            
            // Avoid retouch the edge
            touched[*eit] = 1;
            touched[hew.opp().halfedge()] = 1;
        }
    }
    
    /*
     3. Area force
     */
    for (auto fid = s_dsc->faces_begin(); fid != s_dsc->faces_end(); fid++) {
        auto node_idxs = s_dsc->get_verts(*fid);

        
        std::vector<Vec2> tris = s_dsc->get_pos(*fid);
        int i = 0;
        for(auto ndd : node_idxs){
            tris[i++] = s_dsc->get_pos(ndd);
        }
        double area = s_dsc->area(*fid);
        double ci = mean_inten_[s_dsc->get_label(*fid)];
        
        Vec2 min(INFINITY, INFINITY), max(-INFINITY, -INFINITY);
        for (auto p: tris){
            min[0] = std::min(min[0], p[0]);
            min[1] = std::min(min[1], p[1]);
            max[0] = std::max(max[0], p[0]);
            max[1] = std::max(max[1], p[1]);
        }
        
        double differ = 0.;
        double dd[3] = {0., 0. , 0.};
        for (int i = floor(min[0]); i < ceil(max[0]); i++) {
            for (int j = floor(min[1]); j < ceil(max[1]); j++) {
                if (helper_t::is_point_in_tri(Vec2(i,j), tris)) {
                    differ += std::pow(( (s_img->get_intensity(i, j)) - ci), 2);
                    
                    double xi[3];
                    Vec2 p_c((double)i,(double)j);
                    xi[0] = DSC2D::Util::signed_area(p_c, tris[1], tris[2])/
                            (double)DSC2D::Util::signed_area(tris[0], tris[1], tris[2]);
                    xi[1] = DSC2D::Util::signed_area(tris[0], p_c, tris[2])/
                            DSC2D::Util::signed_area(tris[0], tris[1], tris[2]);
                    xi[2] = DSC2D::Util::signed_area(tris[0], tris[1], p_c)/
                            DSC2D::Util::signed_area(tris[0], tris[1], tris[2]);
                    
                    double aa = std::pow(( (s_img->get_intensity(i, j)) - ci), 2);
                    dd[0] += xi[0]*aa/area;
                    dd[1] += xi[1]*aa/area;
                    dd[2] += xi[2]*aa/area;
                }
            }
        }
        
        differ /= area;
        
        
//        double edge_differ = 0.;
//        auto hew = s_dsc->walker(*fid);
//        for (int k = 0; k < 3; k++) {
//            auto p0 = s_dsc->get_pos(hew.opp().vertex());
//            auto p1 = s_dsc->get_pos(hew.vertex());
//            
//            double cj = mean_inten_[s_dsc->get_label(hew.opp().face())];
//            
//            int length = (int)(p1 - p0).length();
//            
//            for (int k = 0; k <= length; k++) {
//                auto p = p0 + (p1-p0)*(k/(double)length);
//                double I = s_img->get_intensity(p[0], p[1]);
//                edge_differ += -(2*I-ci-cj)*ci;
//            }
//            
//            hew = hew.next();
//        }
    
     //   differ = edge_differ/area;
        
        
        // area grad
        for (int i = 0; i < 3; i++) {
            auto A_idx = node_idxs[i];
            auto B_idx = node_idxs[(i+1)%3];
            auto C_idx = node_idxs[(i+2)%3];
            
            auto A = s_dsc->get_pos(A_idx);
            auto B = s_dsc->get_pos(B_idx);
            auto C = s_dsc->get_pos(C_idx);
            
            double delta = DSC2D::Util::dot(A-B, C-B) / ((C-B).length() * (C-B).length());
            Vec2 H = B + (C-B) * delta;
            Vec2 HA = A - H;
            HA.normalize();
            
            Vec2 NAB = B-A;
            NAB = Vec2(-NAB[1], NAB[0]);
            NAB.normalize();
            Vec2 NAC = C-A;
            NAC = Vec2(NAC[1], -NAC[0]);
            NAC.normalize();
            Vec2 N = NAB + NAC;
            N.normalize();
            
            Vec2 bisector;
            Vec2 AB = B-A;AB.normalize();
            Vec2 AC = C-A; AC.normalize();
            bisector = (AB+AC); bisector.normalize();
            
            Vec2 f = (C-B).length() * N * dd[i];
            
         //   Vec2 f = -bisector * differ * sqrt(area);
            
            s_dsc->add_node_force(A_idx, f, AREA_FORCE);
            
            if (A_idx.get_index() == debug_num[0]) {
                printf("%f %f %f %f \n", A[0], B[0], C[0], f[0]);
                printf("%f %f %f %f \n", A[1], B[1], C[1], f[1]);
            }
        }
    }
    
    // Second derivative
    

    double max_move = 0.;
    double max_g = 0.;
    double max_edge = 0.0;
    dE2 = 0;
    for (auto nid = s_dsc->vertices_begin(); nid != s_dsc->vertices_end(); nid++) {
        
        if (s_dsc->is_interface(*nid)) {
            Vec2 ex = s_dsc->get_node_external_force(*nid);
            Vec2 in = - s_dsc->get_node_internal_force(*nid);
            Vec2 gd = -s_dsc->get_node_force(*nid, AREA_FORCE);
            
      //      double e2 = s_dsc->get_node_force(*nid, E2_GRAD)[0] * 20;
            double e2 = 1.;
            
//            Vec2 count = s_dsc->get_node_force(*nid, V_COUNT);
//            ex /= s_dsc->get_node_force(*nid, V_COUNT)[1];
//            in /= s_dsc->get_node_force(*nid, V_COUNT)[0];

            Vec2 f = gd ;
            Vec2 gE = ex + in + gd;
            Vec2 dis = (f  )/e2 ;
            dE2 += DSC2D::Util::dot(gE, dis);
           // Vec2 dis = ex;
            if (max_move < dis.length()) {
                max_move = dis.length();
            }

            
            s_dsc->set_destination(*nid, s_dsc->get_pos(*nid) + dis);
            s_dsc->set_node_external_force(*nid, gd/e2*10.);
            s_dsc->set_node_internal_force(*nid, in/e2*5.);
        }
    }
    
    printf("Max move: %f \n", max_move);
}

void dynamics_mul::optimize_phase(){
    int nb_phase = (int)mean_inten_.size();
    while (true) {
        bool change = false;
        
        for (auto fid = s_dsc->faces_begin(); fid != s_dsc->faces_end(); fid++) {
            int phase = s_dsc->get_label(*fid);
            double ci = mean_inten_[s_dsc->get_label(*fid)];
            double oldE = energy_triangle(*fid, ci, s_dsc->get_label(*fid));
            
            for (int i = 0; i < nb_phase; i++) {
                double cc = mean_inten_[i];
                if (i != phase) {
                    double newE = energy_triangle(*fid, cc, i);
                    if (newE < oldE) {
                        // change phase to i
                        s_dsc->set_label(*fid, i);
                        change = true;
                        break;
                    }
                }
            }
        }
        
        if (!change) {
            break;
        }
    }
}

double dynamics_mul::energy_triangle(HMesh::FaceID fid, double c,  int new_phase){
    auto tris = s_dsc->get_pos(fid);
    
    // Intensity
    Vec2 min(INFINITY, INFINITY), max(-INFINITY, -INFINITY);
    for (auto p: tris){
        min[0] = std::min(min[0], p[0]);
        min[1] = std::min(min[1], p[1]);
        max[0] = std::max(max[0], p[0]);
        max[1] = std::max(max[1], p[1]);
    }
    
    double ET = 0.;
    for (int i = floor(min[0]); i < ceil(max[0]); i++) {
        for (int j = floor(min[1]); j < ceil(max[1]); j++) {
            if (helper_t::is_point_in_tri(Vec2(i,j), tris)) {
                ET += std::pow(( (s_img->get_intensity(i, j)) - c), 2);
            }
        }
    }
    
    // length
    auto hew = s_dsc->walker(fid);
    double length = 0.;
    for (int i = 0; i < 3; i++) {
        if(HMesh::boundary(*s_dsc->mesh, hew.halfedge()))
            continue;
        
        int phase_opp = s_dsc->get_label( hew.opp().face() );
        if(phase_opp != new_phase){
            length += s_dsc->length(hew.halfedge());
        }
        hew = hew.next();
    }
    
    return ET + 0.01*length;
}

#define NB_PHASE 3

void dynamics_mul::update_probability(dsc_obj &dsc, image &img){
    s_img = &img;
    s_dsc = &dsc;
    
    // Init
    static std::vector<XI> XI_f;
    static KPhi k_f(NB_PHASE);
    
    static bool inited = false;
    if (!inited) {
        //
        for (auto fkey : s_dsc->faces()) {
            s_dsc->set_phase_attr(fkey, Vec2(s_dsc->get_label(fkey)), PHASE_PROBABILITY);
        }
        
        for (int i = 0; i < NB_PHASE; i++) {
            XI_f.push_back(XI(i, NB_PHASE));
        }
        
        inited = true;
    }
    
    // Evolution
    // Compute intensity
    
}

void dynamics_mul::update_dsc_implicit(dsc_obj &dsc, image &img){
    s_img = &img;
    s_dsc = &dsc;
    
    
    helper_t::start_timer();
    s_dsc->deform();
    
    
    
//    if(E0_ > 0.){
//        E1_ = get_total_energy(s_dsc, mean_inten_);
//        double deltaE = E1_ - E0_;
//        dt = std::abs(1./2. * (dE_0_*dt) / ( deltaE - dE_0_*dt)*dt);
//        
//        if(dt > 4000)
//            dt = 4000;
////        if(dt < 1000)
////            dt = 1000;
//        printf("dt = %f \n", dt);
//        
//    }
    
    // 1. Update mean intensity
    // map<phase - mean intensity>
    compute_mean_intensity(mean_inten_);
    g_param.mean_intensity = mean_inten_; // For drawing
    
    

    
    // 3. Curvature force
  //  compute_curvature_force_implicit();
    
    
    // 5. Build and solve matrix
    indexing_vertices();
    build_and_solve();
    
    E0_ = get_total_energy(s_dsc, mean_inten_);
}

void dynamics_mul::indexing_vertices()
{
    int idx = 0;
    for(auto vid = s_dsc->vertices_begin(); vid != s_dsc->vertices_end(); vid++){
        if(!HMesh::boundary(*s_dsc->mesh, *vid)){
            s_dsc->set_node_force(*vid, Vec2(idx), INDEX_VERT);
            idx++;
        }
    }
    for(auto vid = s_dsc->vertices_begin(); vid != s_dsc->vertices_end(); vid++){
        if(HMesh::boundary(*s_dsc->mesh, *vid)){
            s_dsc->set_node_force(*vid, Vec2(idx), INDEX_VERT);
            idx++;
        }
    }
    assert(idx == s_dsc->get_no_vertices());
    
    idx = 0;
    for (auto fid = s_dsc->faces_begin(); fid != s_dsc->faces_end(); fid++) {
        s_dsc->set_phase_attr(*fid, Vec2(idx++), FACE_IDX);
    }
}

std::vector<int> dynamics_mul::get_vert_idx(std::vector<HMesh::VertexID> vids){
    std::vector<int> idxs;
    for (auto v : vids) {
        idxs.push_back((int)s_dsc->get_node_force(v, INDEX_VERT)[0]);
    }
    
    return idxs;
}

void dynamics_mul::build_and_solve(){
    double time_[10] = {0.0};
    helper_t::start_timer();
    
    int width = s_img->width();
    int height = s_img->height();
    int nb_pixel = s_img->width() * s_img->height();
    arma::vec C_x(nb_pixel);
    arma::vec C_y(nb_pixel);
    arma::mat A(nb_pixel, s_dsc->get_no_vertices());
    A.fill(0.);
    C_x.fill(0.);
    C_y.fill(0.);
    
    time_[0] = helper_t::get_time_and_start();
//    std::vector<Vec2> E_grad(width*height, Vec2(-INFINITY));
    
    double grad_scale = 1.0;
    double draw_scale = 10.;
    double displace_scale = dt;
    
    /* 
        1. Image gradient force
     */
    double max_grad = 0.0;
    for (auto fid = s_dsc->faces_begin(); fid != s_dsc->faces_end(); fid++) {
        auto node_idxs = s_dsc->get_verts(*fid);
        auto tris = s_dsc->get_pos(*fid);
        std::vector<int> idxs = get_vert_idx(node_idxs);
        
        double ci = mean_inten_[s_dsc->get_label(*fid)];
        
        Vec2 min(INFINITY, INFINITY), max(-INFINITY, -INFINITY);
        for (auto p: tris){
            min[0] = std::min(min[0], p[0]);
            min[1] = std::min(min[1], p[1]);
            max[0] = std::max(max[0], p[0]);
            max[1] = std::max(max[1], p[1]);
        }
        
        
        for (int i = floor(min[0]); i < ceil(max[0]); i++) {
            for (int j = floor(min[1]); j < ceil(max[1]); j++) {
                if (helper_t::is_point_in_tri(Vec2(i,j), tris)) {
                    Vec2 f = s_img->grad(i, j) * (s_img->get_intensity(i, j) - ci) * grad_scale;
                    
//                    double a2d = s_img->grad(i, j).length() + s_img->grad(i, j).length();
//                    double epsilon = 1e-10;
//                    if(a2d < epsilon){
//                        // normalize f
//                        
//                    }else{
//                        f /= a2d;
//                    }

                    
                    int idx = j*width + i;
                    C_x(idx) = f[0];
                    C_y(idx) = f[1];
                    
//                    E_grad[idx] = f;
                    
                    double l = f.length();
                    if (max_grad < l) {
                        max_grad = l;
                    }
                    // Barry centric linear interpolation
                    Vec2 pos = Vec2(i,j) + Vec2(0.5, 0.5);
                    Vec3 barray_coord;
                    for (int k = 0; k < 3; k++) {
                        barray_coord[k] = DSC2D::Util::signed_area(pos,
                                                                   tris[(k+1)%3],
                                                                   tris[(k+2)%3])
                                        / DSC2D::Util::signed_area(tris[0],
                                                                   tris[1],
                                                                   tris[2]);
                        
                        int vidx = idxs[k];
                        A(idx, vidx) = barray_coord[k];
                    } /* For three points */
                    
                } /* If inside */
            } /* For all point in face */
        } /* For all point in face */
    } /* For all faces */
   
    time_[1] = helper_t::get_time_and_start();
    
    /*
        2. Boundary intensity forces
     */
    double max_inten = 0.0;
    HMesh::HalfEdgeAttributeVector<int> touched(s_dsc->get_no_halfedges(), 0);
    
    for (auto eit = s_dsc->halfedges_begin(); eit != s_dsc->halfedges_end(); eit++) {
        if(s_dsc->is_interface(*eit) and !touched[*eit]){
            auto hew = s_dsc->walker(*eit);
            
            double c0 = mean_inten_[s_dsc->get_label(hew.face())];
            double c1 = mean_inten_[s_dsc->get_label(hew.opp().face())];
            
            // Loop on the edge
            auto p0 = s_dsc->get_pos(hew.opp().vertex());
            auto p1 = s_dsc->get_pos(hew.vertex());
            int idx0 = (int)s_dsc->get_node_force(hew.opp().vertex(), INDEX_VERT)[0];
            int idx1 = (int)s_dsc->get_node_force(hew.vertex(), INDEX_VERT)[0];
            
            int length = (int)(p1 - p0).length();
            
            Vec2 L01 = p1 - p0;
            L01.normalize();
            Vec2 N01(L01[1], -L01[0]);
            
            for (int i = 0; i <= length; i++) {
                auto p = p0 + (p1 - p0)*(double(i)/(double)length);
                double I = s_img->get_intensity(p[0], p[1]);
                int pixel_idx = (int)p[1]*width + (int)p[0];
                A.row(pixel_idx).fill(0.); // Optimize later by flag
                
                // Normalize force
                Vec2 f = N01 * ( (c0-c1)*(2*I - c0 - c1))/ 2.0;
                
                // Grad force
                Vec2 grad = s_img->grad(p[0], p[1]) * (2*I - c0 - c1) / 2.0 * grad_scale;
                
                // TODO: use or not
                f += grad;
                
//                double a2d = (c0-c1)*(c0-c1)
//                + s_img->grad(p[0], p[1]).length()*s_img->grad(p[0], p[1]).length()
//                + 2*DSC2D::Util::dot(s_img->grad(p[0], p[1]), N01)*(c0-c1);
//                
//                f /= a2d;
                
                double l = f.length();
                if (max_inten < l) {
                    max_inten = l;
                }
                
                // Barry Centric coordinate
                C_x(pixel_idx) = f[0];
                C_y(pixel_idx) = f[1];
                
//                E_grad[pixel_idx] = f;
                
                A(pixel_idx, idx0) = (p - p1).length() / length;
                A(pixel_idx, idx1) = (p - p0).length() / length;
                
//                if (i == 0) {
//                    s_dsc->set_node_internal_force(hew.opp().vertex(), f*displace_scale*draw_scale);
//                }
//                if (i == length){
//                    s_dsc->set_node_internal_force(hew.vertex(), f*displace_scale*draw_scale);
//                }
            }
            
            // Avoid retouch the edge
            touched[*eit] = 1;
            touched[hew.opp().halfedge()] = 1;
        }
    }
    
    time_[2] = helper_t::get_time_and_start();
    
    /*
        Solve the matrix
     */
    arma::mat A_t = A.t();
    arma::mat A_t_a = (A_t*A);
    arma::mat CC(s_dsc->get_no_vertices(), 2);
    CC.col(0) = A_t*C_x;
    CC.col(1) = A_t*C_y;

    arma::mat B(s_dsc->get_no_vertices(), 2);
    
    time_[3] = helper_t::get_time_and_start();
    
    
    
//    arma::sp_mat Asp(s_dsc->get_no_vertices(), s_dsc->get_no_vertices());
//    int non_zero_count = 0;
//    for (int i = 0; i < s_dsc->get_no_vertices(); i++) {
//        for (int j = 0; j<s_dsc->get_no_vertices(); j++) {
//            if (A_t_a(i,j) != 0) {
//                Asp(i,j) = A_t_a(i,j);
//                non_zero_count ++;
//            }
//        }
//    }
//    arma::spsolve(B, Asp, CC, "lapack");
 
    
    // Sub matrix
//    std::vector<int> index_order(s_dsc->get_no_vertices());
//    std::vector<int> node_pos_in_mat(s_dsc->get_no_vertices());
//    int begin = 0;
//    int end = (int)index_order.size()-1;
//    for (auto nid = s_dsc->vertices_begin(); nid != s_dsc->vertices_end(); nid++) {
//        int idx = (int)s_dsc->get_node_force(*nid, INDEX_VERT)[0];
//        if (s_dsc->is_crossing(*nid) || s_dsc->is_interface(*nid)) {
//            index_order[end] = idx;
//            node_pos_in_mat[idx] = end;
//            end--;
//        }else{
//            index_order[begin] = idx;
//            node_pos_in_mat[idx] = begin;
//            begin++;
//        }
//    }
//    assert(begin < end);
    
    // swap mat
    
    
    
    arma::solve(B, A_t_a, CC);
  
    time_[4] = helper_t::get_time_and_start();
    
    arma::mat C_a = A*B;
    arma::mat k = C_a.t()*C_a;
    dE_0_ = k(0,0)+ k(1,1);
    
    /*
        Deform
     */
    double total = 0.0f;
    double max_move = 0.0f;
    
    for (auto nid = s_dsc->vertices_begin(); nid != s_dsc->vertices_end(); nid++) {
        
        if (s_dsc->is_interface(*nid)) {
            int idx = (int)s_dsc->get_node_force(*nid, INDEX_VERT)[0];
            Vec2 f(B(idx, 0), B(idx, 1));
            Vec2 dis = f*displace_scale;
            s_dsc->set_destination(*nid, s_dsc->get_pos(*nid) + dis);
            s_dsc->set_node_external_force(*nid, dis*draw_scale);
            total += dis.length();
            
      //      printf("%d: %f ;", (int)nid->get_index(), dis.length());
            
            
            if (max_move < dis.length()) {
                max_move = dis.length();
            }
            
            
        }
    }
    
    
//    E_grad0_ = E_grad;
    
    printf("Error: %f; grad: %f; inten: %f; max_move: %f   %f %f %f %f\n",
                            total,
                            max_grad,
                            max_inten,
                            max_move,
           time_[0], //init
           time_[1], // image
           time_[2], // edge
           time_[3], // mul mat
           time_[4] // solve
           );
    

}

void dynamics_mul::compute_image_gradient_force_implicit(std::vector<Vec2> & grad_force){
    
    for (auto fid = s_dsc->faces_begin(); fid != s_dsc->faces_end(); fid++) {
        auto tris = s_dsc->get_pos(*fid);
        double ci = mean_inten_[s_dsc->get_label(*fid)];
        
        Vec2 min(INFINITY, INFINITY), max(-INFINITY, -INFINITY);
        for (auto p: tris){
            min[0] = std::min(min[0], p[0]);
            min[1] = std::min(min[1], p[1]);
            max[0] = std::max(max[0], p[0]);
            max[1] = std::max(max[1], p[1]);
        }
        
        for (int i = floor(min[0]); i < ceil(max[0]); i++) {
            for (int j = floor(min[1]); j < ceil(max[1]); j++) {
                if (helper_t::is_point_in_tri(Vec2(i,j), tris)) {
                    grad_force[j*s_img->width() + i] = s_img->grad(i, j)
                                                        * (s_img->get_intensity(i, j) - ci);
                }
            }
        }
    }
}

double dynamics_mul::optimal_dt(dsc_obj * clone_dsc){
    double E0 = get_total_energy(s_dsc, mean_inten_);
    double E1 = get_total_energy(clone_dsc, mean_inten_);
    double dE0 = energy_gradient_by_moving_distance(s_dsc, mean_inten_);
    
    std::cout << E0 << " " << " " << E1 << " " << E1-E0 << " " << dE0 << " | E0 E1 delta_E dE0" << std::endl;
    
    return -1./2. * dE0 / (E0 - E1 - dE0);
}

void dynamics_mul::debug_optimum_dt_2(){
    
    for (auto ni = s_dsc->vertices_begin(); ni != s_dsc->vertices_end(); ni++) {
        if (s_dsc->is_interface(*ni)) {
           // Vec2 s = (s_dsc->get_node_internal_force(*ni) + s_dsc->get_node_external_force(*ni));
            
            Vec2 s = s_dsc->get_node_external_force(*ni);
            
            // 1. Find furthest movement
            double alpha_max = furthest_move(*ni, s);
            
            if (alpha_max > 1) {
                alpha_max = 1;
            }
            
            // 2. Compute energy change
            double E0 = star_energy(*ni, s_dsc->get_pos(*ni));
            double E1 = star_energy(*ni, s_dsc->get_pos(*ni) + s*alpha_max/2.0);
            double E2 = star_energy(*ni, s_dsc->get_pos(*ni) + s*alpha_max);
            
            CGLA::Mat3x3f A(CGLA::Vec3f(0, 0, 1),
                            CGLA::Vec3f(alpha_max/2.*alpha_max/2., alpha_max/2., 1),
                            CGLA::Vec3f(alpha_max*alpha_max, alpha_max, 1));
            CGLA::Vec3f b(E0, E1, E2);
            CGLA::Mat3x3f A_i = CGLA::invert(A);
            CGLA::Vec3f coes = A_i*b;
            
            double alpha_g = -1.0/2.0 * coes[1] / coes[0];
            
            if (alpha_g < 0) {
                alpha_g = 0;
            }
            if (alpha_g > 1) {
                alpha_g = 1;
            }
            
            s_dsc->set_node_external_force(*ni, s*alpha_g);
        }
    }
}

double dynamics_mul::energy_gradient_by_moving_distance(dsc_obj *obj,
                                                        std::map<int, double>  intensity_map){
    // 1. Intensity u gradient
    double dEu = u_gradient(obj, intensity_map);
    
    // 2. Intensity image gradient
    double dEg = image_gradient_count(obj, intensity_map);
    
    // 3. Curvature
    double dEl = gradient_length(obj);
    
    std::cout << dEu << "  " << dEg << " " << dEl << " | u-grad; image-frad; length-grad" << std::endl;
    
    return dEu + dEl + dEg;
}

double dynamics_mul::gradient_length(dsc_obj *obj){
    double dEl = 0.0;
    
    for (auto eid = obj->halfedges_begin(); eid != obj->halfedges_end(); eid++) {
        auto hew = obj->walker(*eid);
        if (obj->is_interface(*eid)) {
            
            // Loop on the edge
            auto p0 = s_dsc->get_pos(hew.opp().vertex());
            auto p1 = s_dsc->get_pos(hew.vertex());
            
            int length = (int)(p1 - p0).length();
            for (int i = 0; i <= length; i++) {
                auto p = p0 + (p1 - p0)*(double(i)/(double)length);
                
                double K1, K2;
                get_curvature(obj, hew, K1, K2);
                
                double K = K1*(p0-p).length()/(p0-p1).length()
                            + K2*(p1-p).length()/(p0-p1).length();
                
                dEl += std::pow(g_param.alpha*K, 2);
            }
        }
    }
    return dEl;
}
double dynamics_mul::get_curvature(dsc_obj *obj, HMesh::Walker hew0){
    // Find next edge on the boundary
    auto hew1 = hew0.next().opp();
    while (1) {
        if (s_dsc->is_interface(hew1.halfedge())) {
            hew1 = hew1.opp();
            break;
        }
        
        hew1 = hew1.next().opp();
    }
    
    Vec2 p0 = obj->get_pos(hew0.vertex()) - obj->get_pos(hew0.opp().vertex());
    Vec2 p1 = obj->get_pos(hew1.vertex()) - obj->get_pos(hew1.opp().vertex());
    
    Vec2 norm0(p0[1], -p0[0]); norm0.normalize();
    Vec2 norm1(p1[1], -p1[0]); norm1.normalize();
    Vec2 norm = norm0 + norm1; norm.normalize();
    
    
    double l0 = p0.length();
    double l1 = p1.length();
    double angle = std::atan2(CGLA::cross(p0, p1), DSC2D::Util::dot(p0, p1));
    double curvature = angle / (l0/2.0 + l1/2.0);
    
    return curvature;
}

void dynamics_mul::get_curvature(dsc_obj *obj, HMesh::Walker hew, double &Kcur, double &Kpre){
    // Find preveous edge
    auto hew_pre = hew.prev().opp();
    while (1) {
        if (obj->is_interface(hew_pre.halfedge())) {
            hew_pre = hew_pre.opp();
            break;
        }
        
        hew_pre = hew_pre.prev().opp();
    }
    
    Kcur = get_curvature(obj, hew);
    Kpre = get_curvature(obj, hew_pre);
}

double dynamics_mul::image_gradient_count(dsc_obj *obj, std::map<int, double>  intensity_map)
{
    double dEg = 0.0;
    
    for (auto fid = obj->faces_begin(); fid != obj->faces_end(); fid++) {
        auto tris = obj->get_pos(*fid);
        double ci = mean_inten_[obj->get_label(*fid)];
        
        Vec2 min(INFINITY, INFINITY), max(-INFINITY, -INFINITY);
        for (auto p: tris){
            min[0] = std::min(min[0], p[0]);
            min[1] = std::min(min[1], p[1]);
            max[0] = std::max(max[0], p[0]);
            max[1] = std::max(max[1], p[1]);
        }
        
        for (int i = floor(min[0]); i < ceil(max[0]); i++) {
            for (int j = floor(min[1]); j < ceil(max[1]); j++) {
                if (helper_t::is_point_in_tri(Vec2(i,j), tris)) {
                    double I = s_img->get_intensity(i, j);
                    dEg += std::pow( g_param.beta*(I-ci)*s_img->grad(i, j).length() , 2);
                }
            }
        }
    }
    
    return dEg;
}

double dynamics_mul::u_gradient(dsc_obj *obj, std::map<int, double>  intensity_map)
{
    double dEu = 0.0;
    HMesh::HalfEdgeAttributeVector<int> touch(obj->get_no_halfedges(), 0);
    for (auto eid = obj->halfedges_begin(); eid != obj->halfedges_end(); eid++) {
        auto hew = obj->walker(*eid);
        if (!touch[*eid] and obj->is_interface(*eid)) {
            double c0 = mean_inten_[s_dsc->get_label(hew.face())];
            double c1 = mean_inten_[s_dsc->get_label(hew.opp().face())];
            
            // Loop on the edge
            auto p0 = s_dsc->get_pos(hew.opp().vertex());
            auto p1 = s_dsc->get_pos(hew.vertex());
            
            int length = (int)(p1 - p0).length();
            for (int i = 0; i <= length; i++) {
                auto p = p0 + (p1 - p0)*(double(i)/(double)length);
                double I = s_img->get_intensity(p[0], p[1]);
                
                dEu += std::pow(g_param.beta*(2*I - c0 - c1)*(c0 - c1), 2);
            }
        }
        
        touch[*eid] = 1;
        touch[hew.opp().halfedge()] = 1;
    }
    
    return dEu;
}

double dynamics_mul::get_total_energy(dsc_obj *obj, std::map<int, double>  intensity_map){
    double total_length = 0.0;
    HMesh::HalfEdgeAttributeVector<int> touch(obj->get_no_halfedges(), 0);
    for (auto eid = obj->halfedges_begin(); eid != obj->halfedges_end(); eid++) {
        auto hew = obj->walker(*eid);
        if (!touch[*eid] and obj->is_interface(*eid)) {
            total_length += obj->length(*eid);
        }
        
        touch[*eid] = 1;
        touch[hew.opp().halfedge()] = 1;
    }
    
    double E = 0.0;
    for (auto fid = obj->faces_begin(); fid != obj->faces_end(); fid++) {
        double ci = intensity_map[obj->get_label(*fid)];
        auto tris = obj->get_pos(*fid);
        
        Vec2 min(INFINITY, INFINITY), max(-INFINITY, -INFINITY);
        for (auto p: tris){
            min[0] = std::min(min[0], p[0]);
            min[1] = std::min(min[1], p[1]);
            max[0] = std::max(max[0], p[0]);
            max[1] = std::max(max[1], p[1]);
        }
        
        for (int i = floor(min[0]); i < ceil(max[0]); i++) {
            for (int j = floor(min[1]); j < ceil(max[1]); j++) {
                if (helper_t::is_point_in_tri(Vec2(i,j), tris)) {
                    double I = s_img->get_intensity(i, j);
                    E += (I-ci)*(I-ci);
                }
            }
        }
    }
    
    return E;
}

void dynamics_mul::debug_optimum_dt(){
    
    alpha_map_.clear();
    
    for (auto ni = s_dsc->vertices_begin(); ni != s_dsc->vertices_end(); ni++) {
        if (s_dsc->is_interface(*ni)) {
            Vec2 s = (s_dsc->get_node_internal_force(*ni)
                      + s_dsc->get_node_external_force(*ni));
            
            // 1. Find furthest movement
            double alpha_max = furthest_move(*ni, s);
            
            // 2. Compute energy change
            double delta_E = energy_change(*ni, s_dsc->get_pos(*ni) + s * alpha_max);

            // 3. Compute partial derivative of E wrt to alpha
            double ll = curve_length(*ni, s_dsc->get_pos(*ni));
            double grad_E_a_2 = s.length() * s.length() / ll;
            
            // 4. Optimal alpha
            double alpha_g = 1.0/2.0*(2.0*delta_E - alpha_max*grad_E_a_2)/(delta_E - alpha_max*grad_E_a_2);
            
            cout << " " << alpha_g << endl;
            
        }
    }
}

double dynamics_mul::furthest_move(Node_key nid, Vec2 direction){
    double max_move = s_dsc->intersection_with_link(nid, s_dsc->get_pos(nid) + direction);
    return max_move;
}

void dynamics_mul::compute_curvature_force_implicit(){
    for (auto eid = s_dsc->halfedges_begin(); eid != s_dsc->halfedges_end(); eid++) {
        
        if (s_dsc->is_interface(*eid)) {
            auto hew0 = s_dsc->walker(*eid);
            
            // Find next edge on the boundary
            auto hew1 = hew0.next().opp();
            while (1) {
                if (s_dsc->is_interface(hew1.halfedge())) {
                    hew1 = hew1.opp();
                    break;
                }
                
                hew1 = hew1.next().opp();
            }

            
            Vec2 p0 = s_dsc->get_pos(hew0.vertex()) - s_dsc->get_pos(hew0.opp().vertex());
            Vec2 p1 = s_dsc->get_pos(hew1.vertex()) - s_dsc->get_pos(hew1.opp().vertex());
            
            Vec2 norm0(p0[1], -p0[0]); norm0.normalize();
            Vec2 norm1(p1[1], -p1[0]); norm1.normalize();
            Vec2 norm = norm0 + norm1; norm.normalize();
            
            
            double l0 = p0.length();
            double l1 = p1.length();
            double angle = std::atan2(CGLA::cross(p0, p1), DSC2D::Util::dot(p0, p1));
            double curvature = angle / (l0/2.0 + l1/2.0);
            
            s_dsc->add_node_force(hew0.vertex(), -norm*curvature*s_dsc->get_avg_edge_length()*g_param.alpha, IN_BOUND);
        }
    }
}

void dynamics_mul::compute_curvature_force(){
    for (auto eid = s_dsc->halfedges_begin(); eid != s_dsc->halfedges_end(); eid++) {

        if (s_dsc->is_interface(*eid)) {
            auto hew0 = s_dsc->walker(*eid);
            
            // Find next edge on the boundary
            auto hew1 = hew0.next().opp();
            while (1) {
                if (s_dsc->is_interface(hew1.halfedge())) {
                    hew1 = hew1.opp();
                    break;
                }
                
                hew1 = hew1.next().opp();
            }
            
            assert(hew0.halfedge() != hew1.halfedge());
            
            Vec2 p0 = s_dsc->get_pos(hew0.vertex()) - s_dsc->get_pos(hew0.opp().vertex());
            Vec2 p1 = s_dsc->get_pos(hew1.vertex()) - s_dsc->get_pos(hew1.opp().vertex());
            
            Vec2 norm0(p0[1], -p0[0]); norm0.normalize();
            Vec2 norm1(p1[1], -p1[0]); norm1.normalize();
            Vec2 norm = norm0 + norm1; norm.normalize();

#ifdef DEBUG
            assert(norm.length() < 1.1 and norm.length() > 0.9);
#endif
            
            
            double l0 = p0.length();
            double l1 = p1.length();
            double angle = std::atan2(CGLA::cross(p0, p1), DSC2D::Util::dot(p0, p1));
            double curvature = angle / (l0/2.0 + l1/2.0);
            
            s_dsc->add_node_internal_force(
                    hew0.vertex(), -norm*curvature*s_dsc->get_avg_edge_length()*g_param.alpha);
        }
    }
}

void dynamics_mul::displace_dsc_2(){
    for (auto ni = s_dsc->vertices_begin(); ni != s_dsc->vertices_end(); ni++) {
        if (s_dsc->is_interface(*ni)) {
            
            s_dsc->set_destination(*ni, s_dsc->get_pos(*ni) + s_dsc->get_node_external_force(*ni));
        }
    }
    
    s_dsc->deform();
}

void dynamics_mul::displace_dsc(dsc_obj *obj){
    if (!obj) {
        obj = s_dsc;
    }
    dt = 2;
    
    double total = 0.0;
    dE_0_ = 0.0;
    double max_move = 0.0;
    double el = obj->get_avg_edge_length();
    for (auto ni = obj->vertices_begin(); ni != obj->vertices_end(); ni++) {
        Vec2 dis = (obj->get_node_internal_force(*ni) * 0.01
                    + obj->get_node_external_force(*ni));
        
        double d = obj->get_node_force(*ni, STAR_DIFFER)[0];
        
        double differ = 1.0; // std::atan(d/1) * 2 / PI_V1;
        

        if ((obj->is_interface(*ni) or obj->is_crossing(*ni)))
        {
            obj->set_destination(*ni, obj->get_pos(*ni) + dis*dt*differ);
            total += dis.length();
            
            if (max_move < dis.length()) {
                max_move = dis.length();
            }
            
            dE_0_ += -dis.length()*dis.length()*dt;
        }
    }
    
    obj->deform();
}

void dynamics_mul::compute_mean_intensity(std::map<int, double> & mean_inten_o){
    std::map<int, int> num_pixel_array;
    
    for (auto fid = s_dsc->faces_begin(); fid != s_dsc->faces_end(); fid++) {
        int num_pixel = 0;
        double num_inten = 0.0;
        
        auto tris = s_dsc->get_pos(*fid);
        s_img->get_tri_intensity(tris, &num_pixel, &num_inten);
        
        int phase = s_dsc->get_label(*fid);
        if (mean_inten_o.find(phase) != mean_inten_o.end()) {//Existed
            mean_inten_o[phase] += num_inten;
            num_pixel_array[phase] += num_pixel;
        }else{
            num_pixel_array.insert(std::make_pair(phase, num_pixel));
            mean_inten_o.insert(std::make_pair(phase, num_inten));
        }
    }
    
    total_inten_ = mean_inten_o;
    total_pixel = num_pixel_array;
    
    for (auto mit = mean_inten_o.begin(); mit != mean_inten_o.end(); mit++) {
        mit->second /= (double)num_pixel_array[mit->first];
    }
}

void dynamics_mul::compute_intensity_force_implicit(){
    HMesh::HalfEdgeAttributeVector<int> touched(s_dsc->get_no_halfedges(), 0);
    
    for (auto eit = s_dsc->halfedges_begin(); eit != s_dsc->halfedges_end(); eit++) {
        if(s_dsc->is_interface(*eit) and !touched[*eit]){
            auto hew = s_dsc->walker(*eit);
            
            double c0 = mean_inten_[s_dsc->get_label(hew.face())];
            double c1 = mean_inten_[s_dsc->get_label(hew.opp().face())];
            
            // Loop on the edge
            auto p0 = s_dsc->get_pos(hew.opp().vertex());
            auto p1 = s_dsc->get_pos(hew.vertex());
            
            int length = (int)(p1 - p0).length();
            double f0 = 0.0, f1 = 0.0;
            Vec2 fg0(0.0), fg1(0.0);
            for (int i = 0; i <= length; i++) {
                auto p = p0 + (p1 - p0)*(double(i)/(double)length);
                double I = s_img->get_intensity(p[0], p[1]);
                
                // Normalize force
                int normalizedF = 1;
                double f ;
                switch (normalizedF) {
                    case 1:
                        f = ( (c0-c1)*(2*I - c0 - c1)) / ((c0-c1)*(c0-c1));
                        break;
                    case 2:
                        f = ( (c0-c1)*(2*I - c0 - c1)) / std::abs((c0 - c1));
                        break;
                    case 3:
                        f = (c0-c1)*(2*I - c0 - c1);
                        break;
                    default:
                        f = 0.0;
                        break;
                }
                
                // Barry Centric coordinate
                f0 += f*(p-p1).length() / (double)length;
                f1 += f*(p-p0).length() / (double)length;
            }
            
            // Set force
            Vec2 L01 = p1 - p0;
            L01.normalize();
            Vec2 N01(L01[1], -L01[0]);
            
            Vec2 f_x0 = N01*f0;// - fg0;
            Vec2 f_x1 = N01*f1;// - fg1;
            
            s_dsc->add_node_force(hew.opp().vertex(), f_x0*g_param.beta, EX_BOUND);
            s_dsc->add_node_force(hew.vertex(), f_x1*g_param.beta, EX_BOUND);
            
            // Avoid retouch the edge
            touched[*eit] = 1;
            touched[hew.opp().halfedge()] = 1;
        }
    }
}

void dynamics_mul::compute_intensity_force(){
    HMesh::HalfEdgeAttributeVector<int> touched(s_dsc->get_no_halfedges(), 0);
    
    for (auto eit = s_dsc->halfedges_begin(); eit != s_dsc->halfedges_end(); eit++) {
        auto hew = s_dsc->walker(*eit);
        
        if( s_dsc->is_interface(*eit) and
           !touched[*eit])
        {
            
            
            double c0 = mean_inten_[s_dsc->get_label(hew.face())];
            double c1 = mean_inten_[s_dsc->get_label(hew.opp().face())];
            
            // Loop on the edge
            auto p0 = s_dsc->get_pos(hew.opp().vertex());
            auto p1 = s_dsc->get_pos(hew.vertex());
            
            int length = (int)(p1 - p0).length();
            double f0 = 0.0, f1 = 0.0;
            Vec2 fg0(0.0), fg1(0.0);
            for (int i = 0; i <= length; i++) {
                auto p = p0 + (p1 - p0)*(double(i)/(double)length);
                double I = s_img->get_intensity_f(p[0], p[1]);
                
                // Normalize force
                int normalizedF = 3;
                double f ;
                switch (normalizedF) {
                    case 1:
                        f = ( (c0-c1)*(2*I - c0 - c1)) / ((c0-c1)*(c0-c1));
                        break;
                    case 2:
                        f = ( (c0-c1)*(2*I - c0 - c1)) / std::abs((c0 - c1));
                        break;
                    case 3:
                        f = (c0-c1)*(2*I - c0 - c1);
                        break;
                    default:
                        f = 0.0;
                        break;
                }
                
                // Barry Centric coordinate
                f0 += f*(p-p1).length() / (double)length / (double)length;
                f1 += f*(p-p0).length() / (double)length / (double)length;
            }
            
            // Set force
            Vec2 L01 = p1 - p0;
            L01.normalize();
            Vec2 N01(L01[1], -L01[0]);
            
            Vec2 f_x0 = N01*f0;// - fg0;
            Vec2 f_x1 = N01*f1;// - fg1;
            
            s_dsc->add_node_external_force(hew.opp().vertex(), f_x0*g_param.beta);
            s_dsc->add_node_external_force(hew.vertex(), f_x1*g_param.beta);
            
            // Avoid retouch the edge
            touched[*eit] = 1;
            touched[hew.opp().halfedge()] = 1;
        }
    }
}

double dynamics_mul::star_energy(Node_key nid, Vec2 new_pos){

    double E1 = intensity_energy(nid, new_pos);
    double L1 = curve_length(nid, new_pos);
    
 //   return L1*g_param.alpha + E1*g_param.beta;
    return E1;
}

double dynamics_mul::energy_change(Node_key nid, Vec2 new_pos) {
    double dE = 0.0;

    double E0 = intensity_energy(nid, s_dsc->get_pos(nid));
    double E1 = intensity_energy(nid, new_pos);
    
    double L0 = curve_length(nid, s_dsc->get_pos(nid));
    double L1 = curve_length(nid, new_pos);
    
    dE = (L1 - L0)*g_param.alpha + (E1 - E0)*g_param.beta;

    return dE;
}

// intensity different in node link
double dynamics_mul::intensity_energy(Node_key nid, Vec2 new_pos){
    double E = 0.0;
    for (auto hew = s_dsc->walker(nid); !hew.full_circle(); hew = hew.circulate_vertex_cw()) {
        auto fid = hew.face();
        Vec2_array tris;
        tris.push_back(new_pos);
        tris.push_back(s_dsc->get_pos(hew.vertex()));
        tris.push_back(s_dsc->get_pos(hew.next().vertex()));

        double ci = mean_inten_[s_dsc->get_label(fid)];
        
        Vec2 min(INFINITY, INFINITY), max(-INFINITY, -INFINITY);
        for (auto p: tris){
            min[0] = std::min(min[0], p[0]);
            min[1] = std::min(min[1], p[1]);
            max[0] = std::max(max[0], p[0]);
            max[1] = std::max(max[1], p[1]);
        }
        
        for (int i = floor(min[0]); i < ceil(max[0]); i++) {
            for (int j = floor(min[1]); j < ceil(max[1]); j++) {
                if (helper_t::is_point_in_tri(Vec2(i,j), tris)) {
                    double I = s_img->get_intensity(i, j);
                    E += (I - ci)*(I-ci);
                }
            }
        }
    }
    
    return E;
}

// Interface length in node link
double dynamics_mul::curve_length(Node_key nid, Vec2 new_pos){
    double L = 0.0;
    
    for (auto hew = s_dsc->walker(nid); !hew.full_circle(); hew = hew.circulate_vertex_cw()) {
        if (s_dsc->is_interface(hew.halfedge())) {
            L += (s_dsc->get_pos(hew.vertex()) - new_pos).length();
        }
    }
    
    return L;
}
