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

void adapt_mesh::split_face(DSC2D::DeformableSimplicialComplex &dsc, image &img)
{
    dsc_ = & dsc;
    
    // Face total variation
    double thread = 0.01; // Potential for face splitting and relabeling
    HMesh::FaceAttributeVector<double> intensity(dsc_->get_no_faces(), 0);
    std::vector<Face_key> to_split;
    for (auto fkey : dsc_->faces())
    {
        auto tris = dsc_->get_pos(fkey);
        auto area = dsc_->area(fkey);
        double ci = g_param.mean_intensity[dsc_->get_label(fkey)];
        
        double sum = img.get_sum_on_tri_differ(tris, ci) / area;
        
        intensity[fkey] = sum;
        if (sum > thread and area > 10)
        {
            to_split.push_back(fkey);
        }
    }
    
    // Split high energy face
    double flip_thres = 0.02;
    for (auto fkey : to_split)
    {
        if (intensity[fkey] > flip_thres) {
            // Consider flipping
            int min_label = -1;
            double min_differ = INFINITY;
            auto c_array = g_param.mean_intensity;
            auto tris = dsc_->get_pos(fkey);
            auto area = dsc_->area(fkey);
            
            for (int i = 0; i < c_array.size(); i++) {
                double ci = c_array[i];
                double c_sum = img.get_sum_on_tri_differ(tris, ci) / area;
                
                if (c_sum < min_differ) {
                    min_differ = c_sum;
                    min_label = i;
                }
            }
            
     //       assert(min_label != -1 and min_label != dsc_->get_label(fkey));
            dsc_->set_label(fkey, min_label);
        }else{
            dsc_->split(fkey);
        }
    }
    
    dsc_->clean_attributes();
}

struct edge_s_e
{
    edge_s_e(Edge_key ekey_, double length_):ekey(ekey_), length(length_){}
    Edge_key ekey;
    double length;
};

void adapt_mesh::split_edge(DSC2D::DeformableSimplicialComplex &dsc, image &img)
{
    if(0)
    {
        dsc_ = & dsc;
        
        // Face total variation
        HMesh::FaceAttributeVector<double> intensity(dsc.get_no_faces(), 0);
        for (auto fkey : dsc.faces())
        {
            auto tris = dsc.get_pos(fkey);
    //        auto sum = img.get_sum_on_tri_variation(tris, 2);
            
            auto area = dsc.area(fkey);
            double ci = g_param.mean_intensity[dsc.get_label(fkey)];
            double sum = img.get_tri_differ(tris, ci).total_differ / area;
            
            intensity[fkey] = sum;
        }
        
        double thres = 0.04;
        double sortest_e = 10;
        
        std::vector<edge_s_e> edges;
        for(auto hei = dsc.halfedges_begin(); hei != dsc.halfedges_end(); ++hei)
        {
            if (dsc.is_interface(*hei))
            {
                auto hew = dsc.walker(*hei);
                if(dsc.is_movable(*hei)
                   and dsc.get_label(hew.face()) < dsc.get_label(hew.opp().face()))
                {
                    double ev = intensity[hew.face()] + intensity[hew.opp().face()];
                    ev = ev / std::pow(g_param.mean_intensity[dsc.get_label(hew.face())]
                               - g_param.mean_intensity[dsc.get_label(hew.opp().face())], 2);
                    
                    
                    if (ev > thres
                        and dsc.length(*hei) > sortest_e)
                    {
                        edges.push_back(edge_s_e(*hei, dsc_->length(*hei)));
                    }
                }
            }
        }
        
        // Split long edge fist
        for (auto e: edges)
        {
            dsc_->split(e.ekey);
        }
    }
    
    if(1)
    {
    double thres = 0.12;
    
    std::vector<Edge_key> edges;
    for(auto hei = dsc.halfedges_begin(); hei != dsc.halfedges_end(); ++hei)
    {
        if (dsc.is_interface(*hei)) {
            auto hew = dsc.walker(*hei);
            if(dsc.is_movable(*hei)
               and dsc.get_label(hew.face()) < dsc.get_label(hew.opp().face()))
            {
                edges.push_back(*hei);
            }
        }
    }
    
    auto mean_inten_ = g_param.mean_intensity;
    
    for (auto ekey : edges){
        auto hew = dsc.walker(ekey);
        
        double ev = 0;
        double c0 = mean_inten_[dsc.get_label(hew.face())];
        double c1 = mean_inten_[dsc.get_label(hew.opp().face())];
        
        // Loop on the edge
        auto p0 = dsc.get_pos(hew.opp().vertex());
        auto p1 = dsc.get_pos(hew.vertex());
//        int length = (int)(p1 - p0).length();
//        for (int i = 0; i <= length; i++) {
//            auto p = p0 + (p1 - p0)*(double(i)/(double)length);
//            double I = img.get_intensity_f(p[0], p[1]);
//            
//            // Normalize force
//            double f = (2*I - c0 - c1) / (c0-c1);
//            
//            ev += std::abs(f);
//        }
        double length = (p1 - p0).length();
        int N = (int)length;
        double dl = length/(double)N;
        for (int i = 0; i <= N; i++) {
            auto p = p0 + (p1 - p0)*(i/(double)N)*dl;
            double I = img.get_intensity_f(p[0], p[1]);
            
            // Normalize force
            double f = (2*I - c0 - c1) / (c0-c1);
            
            ev += std::abs(f)*dl;
        }
        
        ev = ev / (length + 5);
        
        if (ev > thres) {
            dsc.split(ekey);
        }
    }
    }
}

void adapt_mesh::split_single_edge(Edge_key ekey)
{
    auto hew = dsc_->walker(ekey);
    
    // Add point to adjencent triangle that has small cap
    add_point_if_need(hew);
    add_point_if_need(hew.opp());
}

void adapt_mesh::add_point_if_need(HMesh::Walker hew)
{
    auto p1 = dsc_->get_pos(hew.opp().vertex());
    auto p2 = dsc_->get_pos(hew.vertex());
    auto p0 = dsc_->get_pos(hew.next().vertex());
    
    if(DSC2D::Util::cos_angle(p1, p0, p2) > std::cos(60*180/PI_V1))
    {
        // Add point
        
    }
}







