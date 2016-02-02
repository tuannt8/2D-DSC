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

    double flip_thres = SPLIT_FACE_COEFFICIENT*mincij*mincij;
    std::cout << "Split thres = " << flip_thres << "; mincij = " << mincij << std::endl;
    
    
    HMesh::FaceAttributeVector<double> variation(dsc_->get_no_faces(), 0);
    std::vector<Face_key> to_split;
    for (auto fkey : dsc_->faces())
    {
        
        auto pts = dsc_->get_pos(fkey);
//        /*
//         Shrink the triangle
//         */
//        auto center = (pts[0] + pts[1] + pts[2]) / 3.0;
//        double aa = 1 - 0.2;
//        pts[0] = center + (pts[0] - center)*aa;
//        pts[1] = center + (pts[1] - center)*aa;
//        pts[2] = center + (pts[2] - center)*aa;
//        /**/
        
        double area;
        double mi = img.get_tri_intensity_f(pts, &area); mi /= area;
        double e = img.get_tri_differ_f(pts, mi)/ (area + SINGULAR_AREA);
        
        variation[fkey] = e;
        if (e < flip_thres)
        {
      //      auto area = dsc_->area(fkey);
//            if (area < SMALLEST_SIZE*SMALLEST_SIZE / 2.0)
//            {
//                cout << "Too small trinagle, not relabeled" << endl;
//                continue;
//            }
            // Consider flipping
            int min_label = -1;
            double min_differ = INFINITY;

            auto tris = dsc_->get_pos(fkey);
       //     auto area = dsc_->area(fkey);
            
            
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
            else
            {
                // Consider collapsing the face
                
            }
        }else{
        //    auto area = dsc_->area(fkey);
            if (area < SMALLEST_SIZE*SMALLEST_SIZE / 2.0)
            {
                continue;
            }

            // Only split stable triangle
            // Triangle with 3 stable edge
            int bStable = 0;
            for (auto w = dsc_->walker(fkey); !w.full_circle(); w = w.circulate_face_ccw())
            {
                bStable += dsc_->bStable[w.vertex()];
            }
          //  cout << "Stable = " << bStable << endl;
            if (bStable > 1)
            {
                
                dsc_->split(fkey);
           //     std::cout << "Adapt: Split face: " << fkey.get_index() << std::endl;
            }
            else
            {
               // cout << "Adapt: not stable " << fkey.get_index() << std::endl;
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

void adapt_mesh::remove_needles(DSC2D::DeformableSimplicialComplex &dsc)
{
    dsc_ = &dsc;
    
    for (auto fit = dsc.faces_begin(); fit != dsc.faces_end(); fit++)
    {
        if (!dsc.mesh->in_use(*fit))
        {
            continue;
        }
        
        HMesh::Walker het = dsc.walker(*fit);
        if (
            dsc.min_angle(*fit) < 10*M_PI/180.// dsc.DEG_ANGLE
            and dsc.max_angle(*fit, het) < 120*M_PI/180.
            )
        {
            dsc.remove_degenerate_needle(*fit);
        }
    }
}

void adapt_mesh::thinning(DSC2D::DeformableSimplicialComplex &dsc, image &img)
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
    
    double flip_thres = SPLIT_FACE_COEFFICIENT*(1-SPLIT_FACE_COEFFICIENT)*mincij*mincij;
    std::cout << "Flip thres = " << flip_thres << "; mincij = " << mincij << std::endl;
    
//    std::vector<Node_key> to_collapse;
    for (auto nkey : dsc_->vertices())
    {
        if (dsc_->is_interface(nkey)
            or HMesh::boundary(*dsc_->mesh, nkey))
        {
            continue;
        }
        
        // Check if the one ring have low variation
        bool low_var = true;
        auto smallest = dsc_->walker(nkey);
        double shortest = INFINITY;
        for (auto hew = dsc_->walker(nkey); !hew.full_circle(); hew = hew.circulate_vertex_ccw())
        {
            auto fkey = hew.face();
            auto pts = dsc_->get_pos(fkey);
            double area;
            double mi = img.get_tri_intensity_f(pts, &area); mi /= area;
            double e = img.get_tri_differ_f(pts, mi)/ (area + SINGULAR_AREA);
            
            if (e > flip_thres)
            {
                low_var = false;
            }
            
            if (dsc_->length(hew.halfedge()) < shortest)
            {
                shortest = dsc_->length(hew.halfedge());
                smallest = hew;
            }
        }
        
        if (low_var)
        {
            dsc_->collapse(smallest.halfedge(), true);
            //dsc.collapse(smallest, 0.0);
        }
    }
    

}

void adapt_mesh::split_edge(DSC2D::DeformableSimplicialComplex &dsc, image &img)
{
    dsc_ = &dsc;
    
    
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
        
        double thres = SPLIT_EDGE_COEFFICIENT*(c0-c1) * (c0-c1);
        
        if (dsc.bStable[hew.vertex()] == 1
            and dsc.bStable[hew.opp().vertex()] == 1)
        {
            if (ev > thres and length > SMALLEST_SIZE) // High energy. Split
            {
                dsc.split_adpat_mesh(ekey);
                cout << "Adapt: Split edge " << ekey.get_index() << "; thres = " << thres <<
                    " energy = " << ev << endl;
            }
            else // Low energy, consider collapse
            {
                // Only collapse edge on the interface
                // And doesnot reduce mesh quality
                // conflict with face split
                
                if(dsc.collapse(ekey, true))
                {
                    cout << "Adapt mesh: Collapse edge " << ekey.get_index() << endl;
                }
            }
        }
    }
}

bool adapt_mesh::collapse_edge(HMesh::Walker hew)
{
    if (dsc_->is_crossing(hew.opp().vertex()) )
    {
        return false;
    }
    
    Vec2 p0 = dsc_->get_pos(hew.opp().vertex());
    Vec2 p1 = dsc_->get_pos(hew.vertex());
    Vec2 p2 = dsc_->get_pos(dsc_->previous_interface(hew).opp().vertex());
    if (DSC2D::Util::cos_angle(p2, p0, p1) < dsc_->COS_MIN_ANGLE)
    {
        return dsc_->collapse(hew, 0.0);
    }
    
    return false;
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






