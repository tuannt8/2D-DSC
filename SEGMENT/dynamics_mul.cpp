//
//  dynamics_mul.cpp
//  DSC
//
//  Created by Tuan Nguyen Trung on 3/20/15.
//  Copyright (c) 2015 Asger Nyman Christiansen. All rights reserved.
//

#include "dynamics_mul.h"

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
    
    // 2. Compute intensity force
    //      External force attributes
    compute_intensity_force();

    
    // 3. Curvature force
    compute_curvature_force();
    

    
    
    s_img = nullptr;
    s_dsc = nullptr;
}

void dynamics_mul::compute_curvature_force(){
    
}

void dynamics_mul::displace_dsc(){
    double el = s_dsc->get_avg_edge_length();
    for (auto ni = s_dsc->vertices_begin(); ni != s_dsc->vertices_end(); ni++) {
        Vec2 dis = (s_dsc->get_node_internal_force(*ni)*g_param.alpha
                    + s_dsc->get_node_external_force(*ni)*g_param.beta) / g_param.mass;
        
        if (dis.length() > 0.0001*el) {
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
                
                double f = (c0-c1)*(2*I - c0 - c1);
                f0 += f*(p-p1).length() / (double)length;
                f1 += f*(p-p0).length() / (double)length;
            }
            
            // Set force
            Vec2 L01 = p1 - p0;
            L01.normalize();
            Vec2 N01(L01[1], -L01[0]);
            
            s_dsc->add_node_external_force(hew.opp().vertex(), N01*f0);
            s_dsc->add_node_external_force(hew.vertex(), N01*f1);
            
            // Avoid retouch the edge
            touched[*eit] = 1;
            touched[hew.opp().halfedge()] = 1;
        }
    }
}