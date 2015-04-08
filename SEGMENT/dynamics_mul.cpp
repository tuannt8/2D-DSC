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

dynamics_mul::dynamics_mul(){
    
}

dynamics_mul::~dynamics_mul(){
    
}

void dynamics_mul::update_dsc(dsc_obj &dsc, image &img){
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
    
    // 4. Debug log
    debug_optimum_dt_2();
    
    s_img = nullptr;
    s_dsc = nullptr;
}

void dynamics_mul::debug_optimum_dt_2(){
    
    for (auto ni = s_dsc->vertices_begin(); ni != s_dsc->vertices_end(); ni++) {
        if (s_dsc->is_interface(*ni)) {
           // Vec2 s = (s_dsc->get_node_internal_force(*ni) + s_dsc->get_node_external_force(*ni));
            
            Vec2 s = s_dsc->get_node_external_force(*ni);
            
            // 1. Find furthest movement
            double alpha_max = furthest_move(*ni, s);
            
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
            
            if(alpha_g < 0)
                alpha_g = 0;
            if (alpha_g > alpha_max) {
                alpha_g = alpha_max;
            }
            
            
            s_dsc->set_node_external_force(*ni, s*alpha_g);
        }
    }
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
//            Vec2 s = (s_dsc->get_node_internal_force(*ni)
//                      + s_dsc->get_node_external_force(*ni));
//            
//            // 1. Find furthest movement
//            double alpha_max = furthest_move(*ni, s);
//            
//            // 2. Compute energy change
//            double E0 = star_energy(*ni, s_dsc->get_pos(*ni));
//            double E1 = star_energy(*ni, s_dsc->get_pos(*ni) + s*alpha_max/2.0);
//            double E2 = star_energy(*ni, s_dsc->get_pos(*ni) + s*alpha_max);
//            
//            CGLA::Mat3x3f A(CGLA::Vec3f(0, 0, 1),
//                            CGLA::Vec3f(alpha_max/2.*alpha_max/2., alpha_max/2., 1),
//                            CGLA::Vec3f(alpha_max*alpha_max, alpha_max, 1));
//            CGLA::Vec3f b(E0, E1, E2);
//            CGLA::Mat3x3f A_i = CGLA::invert(A);
//            CGLA::Vec3f coes = A_i*b;
//            
//            double alpha_g = -1.0/2.0 * coes[1] / coes[0];
//            
//            s_dsc->set_destination(*ni, s_dsc->get_pos(*ni) + s*alpha_g);
            
            s_dsc->set_destination(*ni, s_dsc->get_pos(*ni) + s_dsc->get_node_external_force(*ni));
        }
    }
    
    s_dsc->deform();
}

void dynamics_mul::displace_dsc(){
    for (auto ni = s_dsc->vertices_begin(); ni != s_dsc->vertices_end(); ni++) {
        Vec2 dis = (s_dsc->get_node_internal_force(*ni)
                    + s_dsc->get_node_external_force(*ni)) / g_param.mass;
        
        // It is better if we check is interface instead
       // if (dis.length() > 0.0001*el)
        if (s_dsc->is_interface(*ni))
        {
            s_dsc->set_destination(*ni, s_dsc->get_pos(*ni) + dis);
        }
    }
    
    s_dsc->deform();
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
            
            s_dsc->add_node_external_force(hew.opp().vertex(), N01*f0*g_param.beta);
            s_dsc->add_node_external_force(hew.vertex(), N01*f1*g_param.beta);
            
            // Avoid retouch the edge
            touched[*eit] = 1;
            touched[hew.opp().halfedge()] = 1;
        }
    }
}

double dynamics_mul::star_energy(Node_key nid, Vec2 new_pos){

    double E1 = intensity_energy(nid, new_pos);
    double L1 = curve_length(nid, new_pos);
    
    return L1*g_param.alpha + E1*g_param.beta;
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
