//
//  adapt_mesh.h
//  DSC_seg_integral
//
//  Created by Tuan Nguyen Trung on 7/6/15.
//  Copyright (c) 2015 Asger Nyman Christiansen. All rights reserved.
//

#ifndef __DSC_seg_integral__adapt_mesh__
#define __DSC_seg_integral__adapt_mesh__

#include <stdio.h>
#include "DSC.h"
#include "define.h"
#include "image.h"

class adapt_mesh{
public:
    adapt_mesh();
    ~adapt_mesh();
    
    void split_edge(DSC2D::DeformableSimplicialComplex &dsc, image &img);
    
    void split_face(DSC2D::DeformableSimplicialComplex &dsc, image &img);
private:
    DSC2D::DeformableSimplicialComplex *dsc_;
    void split_single_edge(Edge_key ekey);
    void add_point_if_need(HMesh::Walker hew);
};

#endif /* defined(__DSC_seg_integral__adapt_mesh__) */
