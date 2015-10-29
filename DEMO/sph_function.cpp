//
//  sph_function.cpp
//  DSC
//
//  Created by Tuan Nguyen Trung on 6/16/15.
//  Copyright (c) 2015 Asger Nyman Christiansen. All rights reserved.
//

#include "sph_function.h"
#include "draw.h"
#include "console_debug.h"
#include "../Eigen/Sparse"
#include "object_generator.h"

using namespace HMesh;
using namespace Eigen;

#define sign_(a) (a>0? 1:-1)


DSC2D::vec2 sph_function::Gravity(0,-GRAVITY_ABS);

void sph_function::deform(DSC2D::DeformableSimplicialComplex& dsc){
    dsc_ptr = &dsc;
    
    /*****************************************
     3. Volume lost compensation
     */
    double volumeLost = get_curent_volume() - m_V0;
    
    // 3.1 Index the interface veritces
    HMesh::VertexAttributeVector<int> vIdxs(dsc_ptr->get_no_vertices(), INVALID_IDX);
    int num = 0;
    for (auto vkey : dsc_ptr->vertices())
    {
        if (dsc_ptr->is_interface(vkey) or dsc_ptr->is_crossing(vkey))
        {
            vIdxs[vkey] = num++;
        }
    }
    
    // 3.2 Build the equation
    // We have one equation: M(1, 2*NUM_VERTICES) * X(2*NUM_VERTICES) = volumeLost
    // But we add NUM_VERTICES constraints (to all interface vertices):
    //      - If the vertex is on the circle boundary: It can only move on tangent direction
    //      - If the vertex is free, it moves on normal direction
    // So the total equation is: 1 + NUM_VERTICES
    using EigMat = SparseMatrix<double>;
    using EigVec = VectorXd;
    
    EigMat M(1 + num, 2*num);
    for (auto ekey : dsc_ptr->halfedges())
    {
        if (dsc_ptr->is_interface(ekey))
        {
            // Outter normal of the edge
            auto hew = dsc_ptr->walker(ekey);
            if (dsc_ptr->get_label(hew.face()) != 1)
            {
                hew = hew.opp();
            }
            auto pt_tip = dsc_ptr->get_pos(hew.vertex());
            auto pt_root = dsc_ptr->get_pos(hew.opp().vertex());
            auto line = pt_tip - pt_root;
            double length = line.length();
            
            DSC2D::vec2 norm = DSC2D::Util::normalize(DSC2D::vec2(line[1], -line[0]));
            
            
            // Matrix M
            int idx1 = vIdxs[hew.vertex()];
            int idx2 = vIdxs[hew.opp().vertex()];
            
            M.coeffRef(0, 2*idx1) += norm[0]*length / 2.0;
            M.coeffRef(0, 2*idx1+1) += norm[1]*length / 2.0;
            M.coeffRef(0, 2*idx2) += norm[0]*length / 2.0;
            M.coeffRef(0, 2*idx2+1) += norm[1]*length / 2.0;
        }
    }
    
    EigVec B(1 + num);
    for (int i = 0; i < 1+num; i++)
    {
        B[i] = 0;
    }
    B[0] = -volumeLost;
    
    // 3.3 Apply constraints
    int bound_count = 1;
    for (auto vkey : dsc_ptr->vertices())
    {
        if (dsc_ptr->is_interface(vkey) or dsc_ptr->is_crossing(vkey))
        {
            // Normal of the moving direction. The vertex moves orthogonally to this normal.
            // So the condition is: norm * dv = 0
            DSC2D::vec2 norm;
            if (!is_on_boundary(dsc_ptr->get_pos(vkey), norm))
            {
                auto n = dsc_ptr->get_normal(vkey);
                norm = DSC2D::vec2(n[1], -n[0]);
            }
            
            int idx = vIdxs[vkey];
            M.coeffRef(bound_count, 2*idx) = norm[0];
            M.coeffRef(bound_count, 2*idx+1) = norm[1];
            
            bound_count++;
        }
    }
    
    // Solve
    EigMat M_t = M.transpose();
    EigMat MtM = M_t*M;
    EigVec MtB = M_t*B;
    
    ConjugateGradient<EigMat> cg(MtM);
    EigVec X = cg.solve(MtB);
    
    // Apply displcement
    ver_dis.resize(dsc_ptr->get_no_vertices());
    for (auto vkey : dsc_ptr->vertices())
    {
        int idx = vIdxs[vkey];
        if (idx != INVALID_IDX)
        {
            DSC2D::vec2 dis(X[2*idx], X[2*idx+1]);
            dsc_ptr->set_destination(vkey, dsc_ptr->get_pos(vkey) + dis);
        }
    }
    
    dsc_ptr->deform();
    
    
    /****************************************
     1. Gravity force
     */
    HMesh::VertexAttributeVector<DSC2D::vec2> vels(dsc_ptr->get_no_vertices(), DSC2D::vec2(0.0));
    for(auto vi = dsc.vertices_begin(); vi != dsc.vertices_end(); ++vi)
    {
        if (dsc.is_interface(*vi) or dsc.is_crossing(*vi))
        {
            vels[*vi] += Gravity;
        }
    }
    
    /*****************************************
     2. Curvature force
     */
    for (auto hekey : dsc_ptr->halfedges())
    {
        if (dsc_ptr->is_interface(hekey))
        {
            auto hew = dsc_ptr->walker(hekey);
            auto l = dsc_ptr->get_pos(hew.opp().vertex()) - dsc_ptr->get_pos(hew.vertex());
            l.normalize();
            vels[hew.vertex()] += l*1.1;
        }
    }
    
    
    for(auto vi = dsc.vertices_begin(); vi != dsc.vertices_end(); ++vi)
    {
        if(dsc.is_movable(*vi))
        {
            if (dsc.is_interface(*vi) or dsc.is_crossing(*vi))
            {
                auto new_pos = dsc.get_pos(*vi) + vels[*vi];
                DSC2D::vec2 p_v;
                // Project to circle boundarys
                if (is_outside(new_pos, p_v))
                {
                    new_pos = p_v;
                }
                
                dsc.set_destination(*vi, new_pos);
            }
        }
    }
    dsc.deform();

}

bool sph_function::is_on_boundary(DSC2D::vec2 pt, DSC2D::vec2 &norm)
{
    double l = (pt - center_bound).length();
    if (l > r_bound - 1e-2)
    {
        norm = DSC2D::Util::normalize(pt-center_bound);
        return true;
    }
    
    return false;
}

bool sph_function::is_outside(DSC2D::vec2 pt, DSC2D::vec2 &projectionPoint)
{
    double l = (pt - center_bound).length();
    if (l > r_bound)
    {
        projectionPoint = center_bound + DSC2D::Util::normalize(pt-center_bound)*r_bound;
        return true;
    }
    
    return false;
}

double sph_function::get_curent_volume(){
    double V = 0;
    for (auto fkey: dsc_ptr->faces())
    {
        if (dsc_ptr->get_label(fkey) == 1)
        {
            V += dsc_ptr->area(fkey);
        }
    }
    
    return V;
}

void sph_function::draw(){
    // Boundary
    int N = 150;
    glBegin(GL_LINE_LOOP);
    glColor3f(1, 0, 0);
    double step = 2*M_PI / (double)N;
    for (int i = 0; i < N; i++)
    {
        double aa = step*i;
        DSC2D::vec2 pt = center_bound + DSC2D::vec2(sin(aa)*r_bound, cos(aa)*r_bound);
        glVertex2d(pt[0], pt[1]);
    }
    
    glEnd();
}

void sph_function::init(){
    using namespace DSC2D;
    // Init inside the DSC, half size
    auto corners = dsc_ptr->get_design_domain()->get_corners();
    auto center = dsc_ptr->get_center();
    vec2 diag = (corners[2] - corners[0])/4;
    auto ld_ = center - diag;
    auto ru_ = center + diag;
    
    DSC2D::vec2 gap(10,10);
    DSC2D::ObjectGenerator::create_square(*dsc_ptr, ld_ - gap, ru_- ld_ + gap, 1);
    
    // Constant volume
    m_V0 = get_curent_volume();
    
    // Boundary
    auto pts = dsc_ptr->get_design_domain()->get_corners();
    
    center_bound = (pts[0] + pts[2]) / 2;
    r_bound = (pts[1] - pts[0]).length() * 0.5 * 0.8;
}