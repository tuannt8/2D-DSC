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

/**
 FUEL CELL
 */
//#define FACE_SPLIT_THRES 0.01   // Variation threshold
//#define EDGE_SPLIT_THRES 0.01   // Energy threshold
//#define SINGULAR_AREA 4.        // In computation of triangle variation
//#define SINGULAR_EDGE 3.0

/**
 HAMSTER - GOMU
 */
//#define FACE_SPLIT_THRES 0.002   // Variation threshold
//#define EDGE_SPLIT_THRES 0.38   // Energy threshold
//#define SINGULAR_AREA 4.        // In computation of triangle variation
//#define SINGULAR_EDGE 3.0

/**
 SOUND IMAGE
 */
//#define FACE_SPLIT_THRES 0.01   // Variation threshold
//#define EDGE_SPLIT_THRES 0.2   // Energy threshold
//#define SINGULAR_AREA 4.        // In computation of triangle variation
//#define SINGULAR_EDGE 10.0

/**
 DENTAL
 */
#define FACE_SPLIT_THRES 0.01   // Variation threshold
#define EDGE_SPLIT_THRES 0.4   // Energy threshold
#define SINGULAR_AREA 30.        // In computation of triangle variation
#define SINGULAR_EDGE 10.0

#define SPLIT_FACE_COEFFICIENT  0.2

class adapt_mesh{
public:
    adapt_mesh();
    ~adapt_mesh();
    
    // Split edge manually
    void split_edge(DSC2D::DeformableSimplicialComplex &dsc, image &img);
    
    // Split face mannually
    void split_face(DSC2D::DeformableSimplicialComplex &dsc, image &img);
    
    // remove steiner vertices
    void thinning(DSC2D::DeformableSimplicialComplex &dsc, image &img);
    
    // First step: Split face and relabeling. Not working.
    void split_face_and_relabel(DSC2D::DeformableSimplicialComplex &dsc, image &img);
private:
    DSC2D::DeformableSimplicialComplex *dsc_;
    void split_single_edge(Edge_key ekey);
    void add_point_if_need(HMesh::Walker hew);
    bool collapse_edge(HMesh::Walker hew);
};

#endif /* defined(__DSC_seg_integral__adapt_mesh__) */
