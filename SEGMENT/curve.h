//
//  curve.h
//  DSC
//
//  Created by Tuan Nguyen Trung on 2/12/15.
//  Copyright (c) 2015 Asger Nyman Christiansen. All rights reserved.
//

#ifndef __DSC__curve__
#define __DSC__curve__

#include <stdio.h>
#include "define.h"
#include "DSC.h"
#include "texture_helper.h"

// Parametric curve
class curve:public std::vector<DSC2D::DeformableSimplicialComplex::node_key>{
public:
    typedef DSC2D::DeformableSimplicialComplex::node_key node_key;
    

private:
    std::vector<Vec2> second_derivative_;
    std::vector<Vec2> forth_derivative_;
    
    double m_in_, m_out_; // Mean intensity iinside and outside
public:
    curve();
    ~curve();
    
    // Draw the curve
    void draw(DSC2D::DeformableSimplicialComplex &dsc);
    
    // Compute second and forth derivative
    void update_derivative(DSC2D::DeformableSimplicialComplex &dsc);
    void update_mean_intensity(dsc_obj &complex, texture_helper &tex);
    
    // Get derivative
    Vec2 derive2(int idx){
        return second_derivative_[idx];
    };
    Vec2 derive4(int idx){
        return forth_derivative_[idx];
    };
    double m_in(){return m_in_;}
    double m_out(){return m_out_;}
    
private:
    node_key cycle_at(int idx);
};

#endif /* defined(__DSC__curve__) */
