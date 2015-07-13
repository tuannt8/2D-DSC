//
//  adapt_mesh.cpp
//  DSC_seg_integral
//
//  Created by Tuan Nguyen Trung on 7/6/15.
//  Copyright (c) 2015 Asger Nyman Christiansen. All rights reserved.
//

#include "adapt_mesh.h"

adapt_mesh::adapt_mesh(){
    
}

adapt_mesh::~adapt_mesh(){
    
}

void adapt_mesh::split_face(DSC2D::DeformableSimplicialComplex &dsc, image &img){
    dsc_ = & dsc;
    
    // Face total variation
    double thread = 0.03;
    HMesh::FaceAttributeVector<double> intensity(dsc_->get_no_faces(), 0);
    std::vector<Face_key> to_split;
    for (auto fkey : dsc_->faces()){
        auto tris = dsc_->get_pos(fkey);
        // auto sum = img.get_sum_on_tri_variation(tris, 2);
        
        auto area = dsc_->area(fkey);
        double ci = g_param.mean_intensity[dsc_->get_label(fkey)];
        double sum = img.get_sum_on_tri_differ(tris, ci) / area;
        
        intensity[fkey] = sum;
        if (sum > thread
            and area > 10) {
            to_split.push_back(fkey);
        }
    }
    
    // Split high energy face
    for (auto fkey : to_split) {
        dsc_->split(fkey);
    }
}

struct edge_s_e{
    edge_s_e(Edge_key ekey_, double length_):ekey(ekey_), length(length_){}
    Edge_key ekey;
    double length;
};

void adapt_mesh::split_edge(DSC2D::DeformableSimplicialComplex &dsc, image &img){
    
    dsc_ = & dsc;
    
    // Face total variation
    HMesh::FaceAttributeVector<double> intensity(dsc.get_no_faces(), 0);
    for (auto fkey : dsc.faces()){
        auto tris = dsc.get_pos(fkey);
       // auto sum = img.get_sum_on_tri_variation(tris, 2);
        
        auto area = dsc.area(fkey);
        double ci = g_param.mean_intensity[dsc.get_label(fkey)];
        double sum = img.get_tri_differ(tris, ci).total_differ / area;
        
        intensity[fkey] = sum;
    }
    
    double thres = 0.08;
    
    std::vector<edge_s_e> edges;
    for(auto hei = dsc.halfedges_begin(); hei != dsc.halfedges_end(); ++hei)
    {
        if (dsc.is_interface(*hei)) {
            auto hew = dsc.walker(*hei);
            if(dsc.is_movable(*hei)
               and dsc.get_label(hew.face()) < dsc.get_label(hew.opp().face()))
            {
                double ev = intensity[hew.face()] + intensity[hew.opp().face()];
                //  ev = ev / dsc.length(ekey);
                
                if (ev > thres
                    and dsc.length(*hei) > 5) {
                    edges.push_back(edge_s_e(*hei, dsc_->length(*hei)));
                }

            }
        }
    }
    
    // Split long edge fist
    for (auto e: edges){
        dsc_->split(e.ekey);
    }
}

void adapt_mesh::split_single_edge(Edge_key ekey){
    auto hew = dsc_->walker(ekey);
    
    // Add point to adjencent triangle that has small cap
    add_point_if_need(hew);
    add_point_if_need(hew.opp());
}

void adapt_mesh::add_point_if_need(HMesh::Walker hew){
    auto p1 = dsc_->get_pos(hew.opp().vertex());
    auto p2 = dsc_->get_pos(hew.vertex());
    auto p0 = dsc_->get_pos(hew.next().vertex());
    
    if(DSC2D::Util::cos_angle(p1, p0, p2) > std::cos(60*180/PI_V1))
    {
        // Add point
        
    }
}







