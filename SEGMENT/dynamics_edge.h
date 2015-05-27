//
//  dynamics_edge.h
//  DSC_seg_integral
//
//  Created by Tuan Nguyen Trung on 5/27/15.
//  Copyright (c) 2015 Asger Nyman Christiansen. All rights reserved.
//

#ifndef __DSC_seg_integral__dynamics_edge__
#define __DSC_seg_integral__dynamics_edge__

#include <stdio.h>
#include "define.h"
#include "image.h"

class dynamics_edge{
public:
    dynamics_edge(){};
    ~dynamics_edge(){};

public:
    dsc_obj *dsc_;
    image *img_;

    std::map<int, double> mean_inten_;
public:
    void update_dsc(dsc_obj &dsc, image &img);

    void compute_mean_intensity(std::map<int, double> map);

    void compute_edge_force();
};

#endif /* defined(__DSC_seg_integral__dynamics_edge__) */
