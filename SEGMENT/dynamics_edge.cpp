//
//  dynamics_edge.cpp
//  DSC_seg_integral
//
//  Created by Tuan Nguyen Trung on 5/27/15.
//  Copyright (c) 2015 Asger Nyman Christiansen. All rights reserved.
//

#include "dynamics_edge.h"


void dynamics_edge::update_dsc(dsc_obj &dsc, image &img){
    dsc_ = &dsc;
    img_ = &img;

    /*
     Mean intensity
     */
    compute_mean_intensity(mean_inten_);

    /*
    * Compute the force
    */
    compute_edge_force();


}

void dynamics_edge::compute_mean_intensity(std::map<int, double> mean_inten_o) {
    std::map<int, int> num_pixel_array;

    auto s_dsc = dsc_;
    auto s_img = img_;

    for (auto fid = s_dsc->faces_begin(); fid != s_dsc->faces_end(); fid++) {
        int num_pixel = 0;
        double num_inten = 0.0;

        auto tris = s_dsc->get_pos(*fid);
        s_img->get_tri_intensity(tris, &num_pixel, &num_inten);

        int phase = s_dsc->get_label(*fid);
        if (mean_inten_o.find(phase) != mean_inten_o.end()) {//Existed
            mean_inten_o[phase] += num_inten;
            num_pixel_array[phase] += num_pixel;
        }else{
            num_pixel_array.insert(std::make_pair(phase, num_pixel));
            mean_inten_o.insert(std::make_pair(phase, num_inten));
        }
    }

    for (auto mit = mean_inten_o.begin(); mit != mean_inten_o.end(); mit++) {
        mit->second /= (double)num_pixel_array[mit->first];
    }
}

void dynamics_edge::compute_edge_force() {
    for (Edge_key ekey : dsc_->halfedges()){
        // Loop one only
        auto hew = dsc_->walker(ekey);
        if(hew.vertex().get_index() > hew.opp().vertex().get_index()){
            continue;
        }


    }
}