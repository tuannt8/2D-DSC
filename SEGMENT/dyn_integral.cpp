//
//  dyn_integral.cpp
//  DSC_seg_integral
//
//  Created by Tuan Nguyen Trung on 5/12/15.
//  Copyright (c) 2015 Asger Nyman Christiansen. All rights reserved.
//

#include "dyn_integral.h"
#include "helper.h"
#include <armadillo>
#include <CGLA/Mat2x2f.h>

using namespace std;

double get_total_energy(dsc_obj *obj, image* s_img, std::map<int, double>  intesity_map){
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
        double ci = intesity_map[obj->get_label(*fid)];
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

void dyn_integral:: update_dsc(dsc_obj &dsc, image &img){
    s_dsc = &dsc;
    s_img = &img;
    

    
    
    /*
     Mean intensity
     */
    compute_mean_intensity(mean_inten_);
    
    optimize_phase();
    
    s_dsc->deform();
    
    compute_mean_intensity(mean_inten_);
    g_param.mean_intensity = mean_inten_;
    
//    double E = get_total_energy(s_dsc, s_img, mean_inten_);
//    static int iter = 0;
//    printf("%d %f \n", iter++, E);
    
    /*
     Energy when move the vertex
     */
    compute_derivative();
    
    /*
     Displace
     */
//    displace_dsc();
    
}

void dyn_integral::optimize_phase(){
    for(auto fkey : s_dsc->faces()){
        auto pts = s_dsc->get_pos(fkey);
        
        int cur_label = s_dsc->get_label(fkey);
        int total_pixel;
        double cur_energy = tri_energy_with_phase_assumtion(fkey, cur_label);
        
        int min_phase = -1;
        double min_differ = INFINITY;
        for (int i = 0; i < mean_inten_.size(); i++) {
            double ener = tri_energy_with_phase_assumtion(fkey, i);
            if (min_differ > ener - cur_energy) {
                min_differ = ener - cur_energy;
                min_phase = i;
            }
        }
        assert(min_phase != -1);
        
        
        if (cur_label != min_phase) {
            s_dsc->set_label(fkey, min_phase);
        }
    }
}

double dyn_integral::tri_energy_with_phase_assumtion(Face_key fkey, int assume_phase)
{
    double E = 0.0;
    
    int cur_phase = s_dsc->get_label(fkey);
    
    double length = 0;
    for (auto hew = s_dsc->walker(fkey); !hew.full_circle();
         hew = hew.circulate_face_cw())
    {
        if(HMesh::boundary(*s_dsc->mesh, hew.halfedge()))
        {
            continue;
        }
        
        double opp_label = s_dsc->get_label(hew.opp().face());
        if (opp_label != assume_phase) {
            length += s_dsc->length(hew.halfedge());
        }
    }
    
    int total_pixel;
    double total_differ;
    auto pts = s_dsc->get_pos(fkey);
    s_img->get_tri_differ(pts, &total_pixel, &total_differ, mean_inten_[assume_phase]);
    
    E = total_differ + length*0.01;

    
    return E;
}


void dyn_integral::displace_dsc(){
    
    
    for(auto nkey : s_dsc->vertices()){
        if (s_dsc->is_interface(nkey) || s_dsc->is_crossing(nkey)) {
            // TODO: Compute the force
            Vec2 dE = s_dsc->forces[nkey][FIRST_DERIVE];
            Vec2 ddE = s_dsc->forces[nkey][SECOND_DEREIVE];
            ddE[0] = abs(ddE[0]) + 0.01;
            ddE[1] = abs(ddE[1]) + 0.01;
            
            Vec2 f(- dE[0]/ddE[0], - dE[1]/ddE[1]);
      //      Vec2 f(- dE[0], - dE[1]);
            f = f * 0.05;
            
//            double max = 2;
//            double amp = f.length();
//            double scale = atan(amp/0.01) * 2 / PI_V1 * max;
//            f = f* scale;
            
            
            Vec2 des = s_dsc->get_pos(nkey) + f;
            s_dsc->set_destination(nkey, des);
            
            s_dsc->set_node_external_force(nkey, f*10);
        }else{
            s_dsc->set_node_external_force(nkey, Vec2(0.0));
        }
    }

// s_dsc->deform();
}


void dyn_integral::compute_derivative(){
    for(auto nkey : s_dsc->vertices()){
        if (s_dsc->is_interface(nkey) or s_dsc->is_crossing(nkey)) {
            double E0, Ex0, Ex1, Ey0, Ey1;
            double Ex0y0, Ex0y1, Ex1y0, Ex1y1;
            energy_with_location(E0, nkey, Vec2(0.0));
            energy_with_location(Ex0, nkey, Vec2(-epsilon_deriv, 0.0));
            energy_with_location(Ex1, nkey, Vec2(epsilon_deriv, 0.0));
            energy_with_location(Ey0, nkey, Vec2(0.0, -epsilon_deriv));
            energy_with_location(Ey1, nkey, Vec2(0.0, epsilon_deriv));
            
            energy_with_location(Ex0y0, nkey, Vec2(-epsilon_deriv, -epsilon_deriv));
            energy_with_location(Ex0y1, nkey, Vec2(-epsilon_deriv, epsilon_deriv));
            energy_with_location(Ex1y0, nkey, Vec2(epsilon_deriv, -epsilon_deriv));
            energy_with_location(Ex1y1, nkey, Vec2(-epsilon_deriv, epsilon_deriv));
            
            
            // Derivative
            Vec2 dE(0.0), ddE(0.0);
            dE[0] = (Ex1 - Ex0) / (2. * epsilon_deriv) ;
            dE[1] = (Ey1 - Ey0) / (2. * epsilon_deriv);
            ddE[0] = (Ex1 + Ex0 - 2*E0) / (epsilon_deriv*epsilon_deriv);
            ddE[1] = (Ey1 + Ey0 - 2*E0) / (epsilon_deriv*epsilon_deriv);
            
            // ddE_xy
            double dEx_y0 = (Ex1y0 - Ex0y0) / (2. * epsilon_deriv);
            double dEx_y1 = (Ex1y1 - Ex0y1) / (2. * epsilon_deriv);
            double dEy_x0 = (Ex0y1 - Ex0y0) / (2. * epsilon_deriv);
            double dEy_x1 = (Ex1y1 - Ex1y0) / (2. * epsilon_deriv);
            
            double ddExy = (dEy_x1 - dEy_x0) / (2. * epsilon_deriv);
            double ddEyx = (dEx_y1 - dEx_y0) / (2. * epsilon_deriv);
            
            
            
            s_dsc->forces[nkey][FIRST_DERIVE] = dE;
            s_dsc->forces[nkey][SECOND_DEREIVE] = ddE;
            
            Vec2 d = -dE; // Moving direction
            d.normalize();
            double Ed1, Ed0;
            energy_with_location(Ed1, nkey, d*epsilon_deriv);
            energy_with_location(Ed0, nkey, d*(-epsilon_deriv));
            double de2 = (Ed1 + Ed0 - 2*E0)/(epsilon_deriv*epsilon_deriv);
            
            /* 
             * Compute force right now
             */
            
            Vec2 force;
            
            ddE[0] = abs(ddE[0]) + 0.01;
            ddE[1] = abs(ddE[1]) + 0.01;
            
            de2 = abs(de2) + 0.1;
            
      //      force = Vec2(- dE[0]/ddE[0], - dE[1]/ddE[1]);
            force = Vec2(- dE[0]/de2, - dE[1]/de2);
        //    force = Vec2(-dE[0], -dE[1]);
            force *= 0.05;
            
            Vec2 des = s_dsc->get_pos(nkey) + force;
            s_dsc->set_destination(nkey, des);
            s_dsc->set_node_external_force(nkey, force*10);
        }else{
            s_dsc->set_node_external_force(nkey, Vec2(0.0));
        }
    }
    
//    printf("-------------------------------------\n");
}

bool dyn_integral::energy_with_location_1(double &E, Node_key nkey , Vec2 displace, double * real_dis){
    
    Vec2 cur_pos = s_dsc->get_pos(nkey);
    for (auto hew = s_dsc->walker(nkey); !hew.full_circle();
         hew = hew.circulate_face_cw())
    {
        Vec2_array tris;
        tris.push_back(cur_pos + displace);
        tris.push_back(s_dsc->get_pos(hew.next().vertex()));
        tris.push_back(s_dsc->get_pos(hew.prev().vertex()));
        
        
    }
    
    return true;
}

bool dyn_integral::energy_with_location(double &E, Node_key nkey , Vec2 displace, double * real_dis){
    
    // double ep = 1e-5;
    
    // Check if it move out of the star
    Vec2 cur_pos = s_dsc->get_pos(nkey);
    E = 0.;
//    if((displace.length() < ep) or
//       s_dsc->intersection_with_link(nkey, cur_pos + displace) < 1.0)
    if(1) // Error check later
    {
        
        double length = 0.0;
        double differ = 0.0;
        
        for(auto hew = s_dsc->walker(nkey); !hew.full_circle(); hew = hew.circulate_vertex_cw()){
            
            if (s_dsc->is_interface(hew.halfedge())) {
                length += (s_dsc->get_pos(hew.vertex()) - (cur_pos + displace)).length();
            }
            
            Vec2_array tris;
            tris.push_back(cur_pos + displace);
            tris.push_back(s_dsc->get_pos(hew.vertex()));
            tris.push_back(s_dsc->get_pos(hew.next().vertex()));
            int total_pixel = 0;
            double total_differ = 0.0;
            double ci = mean_inten_[s_dsc->get_label(hew.face())];
            s_img->get_tri_differ(tris, &total_pixel, &total_differ, ci);
            
            differ += total_differ;
        }
        
        E += differ*g_param.beta + length*g_param.alpha;
        return true;
        
    }else{
        // TODO: Too large movement
        assert(0);
        return false;
    }
}

void dyn_integral::compute_mean_intensity(std::map<int, double> & mean_inten_o){
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
    
    for (auto mit = mean_inten_o.begin(); mit != mean_inten_o.end(); mit++) {
        mit->second /= (double)num_pixel_array[mit->first];
    }
}