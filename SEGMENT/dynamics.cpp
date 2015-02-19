//
//  dynamics.cpp
//  DSC
//
//  Created by Tuan Nguyen Trung on 2/12/15.
//  Copyright (c) 2015 Asger Nyman Christiansen. All rights reserved.
//

#include "dynamics.h"

bool dynamics:: update_dsc(DSC2D::DeformableSimplicialComplex &dsc, image &img){
    // 1. Process interface vertices
    curve_list_ = extract_curve(dsc);
    
    // 2. Internal forces
    compute_internal_force(curve_list_, dsc);
    
    // 3. External forces
    compute_external_force(curve_list_, dsc, img);
    
    // 4. Compute displacement
    compute_displacement(dsc);
    
    // 5. Update DSC
    deform(dsc);
    
//    // Debug
//    // 1. Process interface vertices
    curve_list_ = extract_curve(dsc);
//    
//    // 2. Internal forces
    compute_internal_force(curve_list_, dsc);
//    
//    // 3. External forces
    compute_external_force(curve_list_, dsc, img);
//    
//    // 4. Compute displacement
//    compute_displacement(dsc);
    
    return  true;
}

void dynamics::deform(DSC2D::DeformableSimplicialComplex& dsc)
{
    auto init_time = std::chrono::system_clock::now();
    update_compute_time(init_time);
    init_time = std::chrono::system_clock::now();
    
    dsc.deform();
    
    update_deform_time(init_time);
}

dynamics::dynamics(): VelocityFunc(0.1, 0.01){
    
}

dynamics::~dynamics(){
    
}

std::vector<curve> dynamics::extract_curve(DSC2D::DeformableSimplicialComplex &dsc){
    curve new_curve;
 /*
    for (auto hei = dsc.halfedges_begin(); hei != dsc.halfedges_end(); ++hei) {
        if (dsc.is_interface(*hei)) {
            auto hew = dsc.walker(*hei);
            
        }
        

        
    }
 */
  for (auto ni = dsc.vertices_begin(); ni != dsc.vertices_end(); ni++) {
        if (dsc.is_interface(*ni)) {
            // Note: We make hard code here
            // TODO: Fix it later
            new_curve.push_back(*ni);
            
            while (1) {
                // Get vertex on same interface
                auto pt_keys = dsc.get_verts(new_curve.back(), true);
                assert(pt_keys.size() == 2);
                
                curve::node_key next_key = get_next_key(new_curve, pt_keys);
                if (next_key != new_curve[0]) {
                    new_curve.push_back(next_key);
                }else{
                    break;
                }
            }
            break; //TODO: hard code
            
        }
    }

    
    std::vector<curve> curve_list;
    curve_list.push_back(new_curve);
    
    return curve_list;
}

curve::node_key dynamics::get_next_key(curve & cu, std::vector<curve::node_key> pt_keys){
    if (cu.size() == 1) {
        return pt_keys[0];
    }else{
        return ( pt_keys[0] == cu[cu.size() - 2] )? pt_keys[1] : pt_keys[0];
    }
    assert(0);
}

void dynamics::draw_curve(DSC2D::DeformableSimplicialComplex &dsc){
    for(auto & c : curve_list_){
        c.draw(dsc);
    }
}

void dynamics::compute_internal_force(std::vector<curve> &curve_list,
                            DSC2D::DeformableSimplicialComplex &dsc){
    curve & cu0 = curve_list[0]; //TODO: Multiple curves
    
    cu0.update_derivative(dsc);
    for (int i = 0; i < cu0.size(); i++) {
        Vec2 force = cu0.derive2(i)*d_param_.alpha - cu0.derive4(i)*d_param_.beta;
        dsc.set_node_internal_force(cu0[i], force);
    }
}

void dynamics::compute_external_force(std::vector<curve> &curve_list
                                      ,dsc_obj &complex
                                      ,image &img){
    curve & cu0 = curve_list[0]; //TODO: Multiple curve
    
    cu0.update_mean_intensity(complex, img);
    
    for (int i = 0; i < cu0.size(); i++) {
 
        
        
        Vec2 pt = complex.get_pos(cu0[i]);
        double inten = img.get_intensity_i(pt[0], pt[1]);
        double scale = (cu0.m_out() - cu0.m_in())* (inten - cu0.m_out() + inten - cu0.m_in());
        //Vec2 norm = complex.get_normal(cu0[i]);
        Vec2 norm = img.get_local_norm(complex, cu0[i], scale < 0);
        Vec2 force = norm*std::abs(scale) * d_param_.gamma;
        
        complex.set_node_external_force(cu0[i], force);
    }
}

void dynamics::compute_displacement(dsc_obj &dsc){
    double max = 0;
    double el = dsc.get_avg_edge_length();
    for (auto ni = dsc.vertices_begin(); ni != dsc.vertices_end(); ni++) {
        Vec2 dis = (dsc.get_node_internal_force(*ni) + dsc.get_node_external_force(*ni))
                        / d_param_.mass;
        if (max < dis.length()) {
            max = dis.length();
        }
        
        if (dis.length() > 0.0001*el) {
            dsc.set_destination(*ni, dsc.get_pos(*ni) + dis);
        }
        
    }
    
    // To force maximum displacement winthin 1 edge length
//    std::cout << "Max displacement: " << max <<", Average edge length: " << el <<std::endl;
    
//    if (max > el) {
//        double scale = el / max;
//        for (auto ni = dsc.vertices_begin(); ni != dsc.vertices_end(); ni++) {
//            Vec2 dis = dsc.get_destination(*ni) - dsc.get_pos(*ni);
//            
//            if (dis.length() > 0.0001*el) {
//                dis = dis * scale;
//                dsc.set_destination(*ni, dsc.get_pos(*ni) + dis);
//            }
//        }
//    }
}