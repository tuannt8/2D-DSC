//
//  adapt_mesh.cpp
//  DSC_seg_integral
//
//  Created by Tuan Nguyen Trung on 7/6/15.
//  Copyright (c) 2015 Asger Nyman Christiansen. All rights reserved.
//

#include "adapt_mesh.h"
#include "dynamics_mul.h"



adapt_mesh::adapt_mesh(){
    
}

adapt_mesh::~adapt_mesh(){
    
}

void adapt_mesh::split_face(DSC2D::DeformableSimplicialComplex &dsc, image &img)
{
    dsc_ = & dsc;
    
    /**
     Auto get threshold
     */
    auto c_array = g_param.mean_intensity;
    double mincij = INFINITY;
    for (int i = 0; i < c_array.size(); i++)
    {
        for (int j = 0; j < c_array.size(); j++)
        {
            if (i != j and mincij > std::abs(c_array[i] - c_array[j]))
            {
                mincij = std::abs(c_array[i] - c_array[j]);
            }
        }
    }
    assert(mincij > 1e-5);

    double flip_thres = 0.08*0.92*mincij*mincij;
    std::cout << "Flip thres = " << flip_thres << "; mincij = " << mincij << std::endl;
    
    
    HMesh::FaceAttributeVector<double> variation(dsc_->get_no_faces(), 0);
    std::vector<Face_key> to_split;
    for (auto fkey : dsc_->faces())
    {

        auto pts = dsc_->get_pos(fkey);
        double area;
        double mi = img.get_tri_intensity_f(pts, &area); mi /= area;
        double e = img.get_tri_differ_f(pts, mi)/ (area + SINGULAR_AREA);
        
        variation[fkey] = e;
        if (e < flip_thres and area > 1)
        {
            
            // Consider flipping
            int min_label = -1;
            double min_differ = INFINITY;

            auto tris = dsc_->get_pos(fkey);
            auto area = dsc_->area(fkey);
            
            
            for (int i = 0; i < c_array.size(); i++) {
                double ci = c_array[i];
                double c_sum = img.get_tri_differ_f(tris, ci) / area;
                
                
                if (c_sum < min_differ) {
                    min_differ = c_sum;
                    min_label = i;
                }
            }
            
            if(min_label != dsc_->get_label(fkey))
                dsc_->update_attributes(fkey, min_label);
        }else{
            // Only split stable triangle
            // Triangle with 3 stable edge
            int bStable = 0;
            for (auto w = dsc_->walker(fkey); !w.full_circle(); w = w.circulate_face_ccw())
            {
                bStable += dsc_->bStable[w.vertex()];
            }
            
            // For testing relabeling in the beginning
            // bStable = 22;
            
            if (bStable > 1)
            {
                dsc_->split(fkey);
                std::cout << "Split: " << fkey.get_index() << std::endl;
            }
        }
    }
    
  //  dsc_->clean_attributes();
}

void adapt_mesh::split_face_and_relabel(DSC2D::DeformableSimplicialComplex &dsc, image &img)
{
//    double thres = 0.05; // ratio to average length
//    while (dsc.get_min_length_ratio() > thres)
    {
        
        dsc.increase_resolution_range();
        dynamics_mul a;
        a.compute_mean_intensity(dsc, img);
        split_face(dsc, img);
        dsc.smooth();
    }
}

struct edge_s_e
{
    edge_s_e(Edge_key ekey_, double length_):ekey(ekey_), length(length_){}
    Edge_key ekey;
    double length;
};

void adapt_mesh::thinning_interface(DSC2D::DeformableSimplicialComplex &dsc, image &img)
{

}

void adapt_mesh::split_edge(DSC2D::DeformableSimplicialComplex &dsc, image &img)
{
    double thres = EDGE_SPLIT_THRES;
    
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
    
    for (auto ekeyp = dsc.halfedges_begin(); ekeyp != dsc.halfedges_end(); ekeyp++){

        auto ekey = *ekeyp;
        auto hew = dsc.walker(ekey);
        
        if (! dsc.mesh->in_use(*ekeyp)
            or !dsc.is_interface(ekey)
            or hew.vertex() < hew.opp().vertex()
            or hew.face() == HMesh::InvalidFaceID
            or hew.opp().face() == HMesh::InvalidFaceID)
        {
            continue;
        }
        

        
        double ev = 0;
        double c0 = mean_inten_[dsc.get_label(hew.face())];
        double c1 = mean_inten_[dsc.get_label(hew.opp().face())];
        
        // Loop on the edge
        auto p0 = dsc.get_pos(hew.opp().vertex());
        auto p1 = dsc.get_pos(hew.vertex());

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
        
        ev = ev / (length + SINGULAR_EDGE);
        
        if (dsc.bStable[hew.vertex()] == 1
            and dsc.bStable[hew.opp().vertex()] == 1)
        {
            if (ev > thres) // High energy. Split
            {
                dsc.split(ekey);
                cout << "Split " << ekey.get_index() << endl;
            }
            else // Low energy, consider collapse
            {
                bool success = dsc.collapse(ekey, true);
                
                
//                if (success)
//                {
//                    cout << "Collapse " << hew.halfedge().get_index() << endl;
//                }
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







