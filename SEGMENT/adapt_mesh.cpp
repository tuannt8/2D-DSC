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

void adapt_mesh::split_edge(DSC2D::DeformableSimplicialComplex &dsc, image &img){
    
    dsc_ = & dsc;
    
    // Face total variation
    HMesh::FaceAttributeVector<double> intensity(dsc.get_no_faces(), 0);
    for (auto fkey : dsc.faces()){
        auto tris = dsc.get_pos(fkey);
        auto sum = img.get_sum_on_tri_variation(tris, 2);
        
        intensity[fkey] = sum;
    }
    
    std::vector<Edge_key> edges;
    for(auto hei = dsc.halfedges_begin(); hei != dsc.halfedges_end(); ++hei)
    {
        if (dsc.is_interface(*hei)) {
            auto hew = dsc.walker(*hei);
            if(dsc.is_movable(*hei) && dsc.get_label(hew.face()) < dsc.get_label(hew.opp().face()))
            {
                edges.push_back(*hei);
            }
        }
    }
    
    double thres = 0.01;
    // Edge total variation
    for (auto ekey : edges){
        auto hew = dsc.walker(ekey);
        
        double ev = intensity[hew.face()] + intensity[hew.opp().face()];
        ev = ev / dsc.length(ekey);
        
        if (ev > thres) {
            //split_single_edge(ekey);
            dsc_->split(ekey);
        }
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







