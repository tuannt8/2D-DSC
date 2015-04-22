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
#include "util.h"

dynamics_mul::dynamics_mul(){
    
}

dynamics_mul::~dynamics_mul(){
    
}

void dynamics_mul::update_dsc_explicit(dsc_obj &dsc, image &img){
    
    s_img = &img;
    s_dsc = &dsc;
    
    // 4. Update DSC
    displace_dsc();
    
    // 1. Update mean intensity
    // <phase - mean intensity>
    compute_mean_intensity(mean_inten_);
    g_param.mean_intensity = mean_inten_; // For drawing
    
    // 2. Compute intensity force
    //      External force attributes
    compute_intensity_force();
    
    // 3. Curvature force
    compute_curvature_force();
}

void dynamics_mul::update_dsc(dsc_obj &dsc, image &img){
    //update_dsc_implicit(dsc, img);
     update_dsc_explicit(dsc, img);
    // update_dsc_explicit_whole_domain(dsc, img);
}

void dynamics_mul::update_dsc_explicit_whole_domain(dsc_obj &dsc, image &img){
    s_img = &img;
    s_dsc = &dsc;
    
    s_dsc->deform();
    
    // 1. Update mean intensity
    // map<phase - mean intensity>
    compute_mean_intensity(mean_inten_);
    g_param.mean_intensity = mean_inten_; // For drawing
    
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
        
//        int num_pixel=  0;
//        for (int i = floor(min[0]); i < ceil(max[0]); i++) {
//            for (int j = floor(min[1]); j < ceil(max[1]); j++) {
//                if (helper_t::is_point_in_tri(Vec2(i,j), tris)) {
//                    num_pixel++;
//                }
//            }
//        }
        
        double area = s_dsc->area(*fid);
        
        for (int i = floor(min[0]); i < ceil(max[0]); i++) {
            for (int j = floor(min[1]); j < ceil(max[1]); j++) {
                if (helper_t::is_point_in_tri(Vec2(i,j), tris)) {
                    Vec2 f = s_img->grad(i, j) * (s_img->get_intensity(i, j) - ci);
                    
//                    double a2d = s_img->grad(i, j).length() + s_img->grad(i, j).length();
//                    double epsilon = 1e-10;
//                    if(a2d < epsilon){
//                        // normalize f
//                        
//                    }else{
//                        f /= a2d;
//                    }
                    
                    
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
    HMesh::HalfEdgeAttributeVector<int> touched(s_dsc->get_no_halfedges(), 0);
    
    for (auto eit = s_dsc->halfedges_begin(); eit != s_dsc->halfedges_end(); eit++) {
        if(s_dsc->is_interface(*eit) and !touched[*eit]){
            auto hew = s_dsc->walker(*eit);
            
            double c0 = mean_inten_[s_dsc->get_label(hew.face())];
            double c1 = mean_inten_[s_dsc->get_label(hew.opp().face())];
            
            double area0 = s_dsc->area(hew.face());
            double area1 = s_dsc->area(hew.opp().face());
            
            // Loop on the edge
            auto p0 = s_dsc->get_pos(hew.opp().vertex());
            auto p1 = s_dsc->get_pos(hew.vertex());
            
            int length = (int)(p1 - p0).length();
            
            Vec2 L01 = p1 - p0;
            L01.normalize();
            Vec2 N01(L01[1], -L01[0]);
            
            for (int i = 0; i <= length; i++) {
                auto p = p0 + (p1 - p0)*(double(i)/(double)length);
                double I = s_img->get_intensity(p[0], p[1]);

                
                // Normalize force
                Vec2 f = N01 * ( (c0-c1)*(2*I - c0 - c1))/ 2.0;
                
               // Vec2 f = N01 * ( (c0-c1)* ( (I - c0)*area0 + (I - c1)*area1 ) );
                
                // Grad force
//                Vec2 grad = s_img->grad(p[0], p[1]) * (2*I - c0 - c1) / 2.0;
//
//                // TODO: use or not
//                f += grad;
//                
//                double a2d = (c0-c1)*(c0-c1)
//                + s_img->grad(p[0], p[1]).length()*s_img->grad(p[0], p[1]).length()
//                + 2*DSC2D::Util::dot(s_img->grad(p[0], p[1]), N01)*(c0-c1);
//                
//                f /= a2d;
                
                double l = f.length();
                if (max_inten < l) {
                    max_inten = l;
                }

                s_dsc->add_node_external_force(hew.opp().vertex(), f*(p - p1).length() / length );
                s_dsc->add_node_external_force(hew.vertex(), f*(p - p0).length() / length );
                
                s_dsc->add_node_force(hew.opp().vertex(), Vec2(0,1), V_COUNT);
                s_dsc->add_node_force(hew.vertex(), Vec2(0,1), V_COUNT);
                
            }
            
            // Avoid retouch the edge
            touched[*eit] = 1;
            touched[hew.opp().halfedge()] = 1;
        }
    }

    double max_move = 0.;
    double max_g = 0.;
    double max_edge = 0.0;
    for (auto nid = s_dsc->vertices_begin(); nid != s_dsc->vertices_end(); nid++) {
        
        if (s_dsc->is_interface(*nid)) {
            Vec2 ex = s_dsc->get_node_external_force(*nid);
            Vec2 in = s_dsc->get_node_internal_force(*nid);
            
//            Vec2 count = s_dsc->get_node_force(*nid, V_COUNT);
//            ex /= s_dsc->get_node_force(*nid, V_COUNT)[1];
//            in /= s_dsc->get_node_force(*nid, V_COUNT)[0];
            
            Vec2 dis = (ex + in)/5.;
            if (max_move < dis.length()) {
                max_move = dis.length();
            }

            
            s_dsc->set_destination(*nid, s_dsc->get_pos(*nid) + dis);
            s_dsc->set_node_external_force(*nid, ex);
            s_dsc->set_node_internal_force(*nid, in);
        }
    }
    printf("Max move: %f \n", max_move);
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
        // Beware of conversion between integer and double
        s_dsc->set_node_force(*vid, Vec2(idx), INDEX_VERT);
        idx++;
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
    double total = 0.0;
    double max_move = 0.0;
    double el = obj->get_avg_edge_length();
    for (auto ni = obj->vertices_begin(); ni != obj->vertices_end(); ni++) {
        Vec2 dis = (obj->get_node_internal_force(*ni)
                    + obj->get_node_external_force(*ni)) / g_param.mass;
        
        // It is better if we check is interface instead
       // if (dis.length() > 0.0001*el)
        if ((obj->is_interface(*ni) or obj->is_crossing(*ni)))
        {
            obj->set_destination(*ni, obj->get_pos(*ni) + dis);
            total += dis.length();
            
            if (max_move < dis.length()) {
                max_move = dis.length();
            }
        }
    }
    
    printf("Max dis: %f\n", max_move);
    
    obj->deform();
}

void dynamics_mul::compute_mean_intensity(std::map<int, double> & mean_inten_o){
    std::map<int, double> num_pixel_array;
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
    
    for (auto mit = mean_inten_o.begin(); mit != mean_inten_o.end(); mit++) {
        mit->second /= num_pixel_array[mit->first];
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
                f0 += f*(p-p1).length() / (double)length;
                f1 += f*(p-p0).length() / (double)length;
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
