//
//  dynamics_prob.cpp
//  DSC_seg
//
//  Created by Tuan Nguyen Trung on 5/6/15.
//  Copyright (c) 2015 Asger Nyman Christiansen. All rights reserved.
//

#include "dynamics_prob.h"

#include "helper.h"

using namespace std;

void dynamics_prob::update_dsc(dsc_obj & obj, image & img){
    dsc_ = & obj;
    img_ = & img;
    
    /* 
     1. xac suat get same value with phase
     */
    init_xac_suat();
    
    /*
     2. Update average C
     */
    compute_c();
    
    /*
     Update triangle intensity
     */
    update_tri_intensity();
    
    /*
     Compute xac suat force
     */
    compute_xac_suat_force();
    
    /*
     update probability
     */
    update_xac_suat();
    
    update_lambda();
}

void dynamics_prob::update_xac_suat(){
//    double max = 0;
//    for (auto fkey : dsc_->faces()) {
//        double f = -dsc_->face_att[fkey][xac_suat_force][0];
//        double area = dsc_->area(fkey);
//        
//        double s = dsc_->face_att[fkey][xac_suat][0];
//        
//        double dis = f/area*dt;
//        dsc_->face_att[fkey][xac_suat] = Vec2(s + dis);
//        
//        if (dis > max) {
//            max = dis;
//        }
//    }
//    
//    printf("Max: %f ; %f x %f \n", max, C_vec(0), C_vec(1));
}

void dynamics_prob::update_lambda(){
    for (auto fkey : dsc_->faces()) {
        double l = dsc_->face_att[fkey][lambda][0];
        double s = dsc_->face_att[fkey][xac_suat][0];
        double k = K_Phi::value(s);
        
        double new_l = l + g_param.rK * k * 0.1;
        dsc_->face_att[fkey][lambda]
            = Vec2(new_l);
    }
}

void dynamics_prob::compute_xac_suat_force(){
    /*
//     intensity
//     */
//    for (auto fkey : dsc_->faces()){
//
//        // Force is independent to each triangle
//
//        double ui = dsc_->face_att[fkey][intensity][0];
//        double differ = 0.0;
//        auto tris = dsc_->get_pos(fkey);
//        Vec2 min(INFINITY, INFINITY), max(-INFINITY, -INFINITY);
//        for (auto p: tris){
//            min[0] = std::min(min[0], p[0]);
//            min[1] = std::min(min[1], p[1]);
//            max[0] = std::max(max[0], p[0]);
//            max[1] = std::max(max[1], p[1]);
//        }
//        for (int i = floor(min[0]); i < ceil(max[0]); i++) {
//            for (int j = floor(min[1]); j < ceil(max[1]); j++) {
//                if (helper_t::is_point_in_tri(Vec2(i,j), tris)) {
//                    differ += 2 * (ui - img_->get_intensity(i, j));
//                }
//            }
//        }
//        
//        double derivative = 0.;
//        double s = dsc_->face_att[fkey][xac_suat][0];
//        for (int i = 0; i < num_phase; i++) {
//            double diff = XI_f::diff(s, i);
//            derivative += diff * C_vec(i);
//        }
//        
//        double area = dsc_->area(fkey);
//        double f1 = differ * derivative;
//        
//        dsc_->face_att[fkey][xac_suat_force] += Vec2(f1);

//    }
    
//    /*
//     Edge force
//     */
//    for (auto fkey : dsc_->faces()){
//        if(fkey.get_index() == 30 or fkey.get_index() == 31){
//            
//        }
//        
//        double f2 = 0.0;
//        for (auto hew = dsc_->walker(fkey); !hew.full_circle(); hew = hew.circulate_vertex_cw()) {
//            if (HMesh::boundary(*dsc_->mesh, hew.halfedge())) {
//                continue;
//            }
//            
//            double length = dsc_->length(hew.halfedge());
//
//            for (int i = 0; i < num_phase; i++) {
//                double s = dsc_->face_att[hew.face()][xac_suat][0];
//                double phi = XI_f::value(s, i);
//                double s_opp = dsc_->face_att[hew.opp().face()][xac_suat][0];
//                double phi_opp = XI_f::value(s_opp, i);
//                
//                double diff = XI_f::diff(s, i);
//                
//                f2 += sign(phi - phi_opp) * length * diff;
//            }
//        }
//        
//        dsc_->face_att[fkey][xac_suat_force] += Vec2(f2);
//    }

    /*
     K
     */
    for (auto fkey : dsc_->faces()){
        if(fkey.get_index() == 30 or fkey.get_index() == 31){
            
        }
        
        double area = dsc_->area(fkey);
        double s = dsc_->face_att[fkey][xac_suat][0];
        double diff = K_Phi::diff(s);
        
        double l = dsc_->face_att[fkey][lambda][0];
        
        double f = diff * l * area;
        
        dsc_->face_att[fkey][xac_suat_force] += Vec2(f);

    }

    /*
     Diff K
     */
    for (auto fkey : dsc_->faces()){
        if(fkey.get_index() == 30 or fkey.get_index() == 31){
            
        }
        double area = dsc_->area(fkey);
        double s = dsc_->face_att[fkey][xac_suat][0];
        double diff = K_Phi::diff(s);
        double k = K_Phi::value(s);
        
        double f = g_param.rK * k * diff * area;
        
        dsc_->face_att[fkey][xac_suat_force] += Vec2(f);

    }
    
    cout<<endl;
}

double K_Phi::value(double s, int np){
    double v = 1.;
    for (int i = 0; i < np; i++) {
        v *= (s - i);
    }
    return v;
}
double K_Phi::diff(double s, int np){
    vector<double> element;
    for (int i = 0; i < np; i++) {
        element.push_back(s - i);
    }
    
    double d = 0.;
    for (int i = 0; i < np; i++) {
        double cur = 1.0;
        for (int j = 0; j < np; j++) {
            if (j != i) {
                cur *= element[j];
            }
        }
        d += cur;
    }
    
    return d;
}

void dynamics_prob::init_xac_suat(){
    static bool xac_suat_inited = false;
    if (!xac_suat_inited) {
        xac_suat_inited = true;
        
        for (auto fkey : dsc_->faces()) {
            dsc_->face_att[fkey][xac_suat]
                = Vec2(dsc_->get_label(fkey));
            
            dsc_->face_att[fkey][lambda]
                = Vec2(g_param.lambda);
        }
    }
}

void dynamics_prob::update_tri_intensity(){

//    for (auto fkey: dsc_->faces()) {
//        double x = dsc_->face_att[fkey][xac_suat][0];
//        
//        double inten = 0.;
//        for (int i = 0; i < num_phase; i++) {
//            double XI_i = XI_f::value(x, i);
//            double C_i = C_vec(i);
//            inten += XI_i * C_i;
//        }
//        dsc_->face_att[fkey][intensity] = Vec2(inten);
//    }
}

void dynamics_prob::compute_c(){


}

#define ep_xi 0.04
double XI_f::value(double x               // xác suất
                   , int p                // Phase
                   , int np   // number of phase
){
    double alpha = 1.0;
    double v = 1.0;
    
    for (int i = 0; i < np; i++){
        if(i != p){
            v *= (x - i);
            alpha *= (p - i);
        }
    }
    return v/alpha;
    
    return 1.0/(1.0 + pow((x-p), 4) / ep_xi);
}

double XI_f::diff(double s               // xác suất
                   , int p                // Phase index
                   , int np   // number of phase);
){
    double v = 1.0;
    double alpha = 1.0;
    vector<double> element;
    
    for (int i = 0; i < np; i++){
        if(i != p){
            element.push_back(s-i);
            v *= (s-i);
            alpha *= (p - i);
        }
    }
    
    double diff_t = 0.0;
    for (int i = 0; i < element.size(); i++) {
        double cur = 1.0;
        for (int j = 0; j < element.size(); j++) {
            if (i != j) {
                cur *= element[j];
            }
        }
        diff_t += cur;
    }
    
    return diff_t / alpha;
    
    return -4.0 * pow(s-p, 3) / ep_xi / pow(1.0 + pow((s-p), 4)  / ep_xi, 2);
}