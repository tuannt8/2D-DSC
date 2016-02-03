//
// Created by Tuan Nguyen Trung on 2/27/15.
// Copyright (c) 2015 Asger Nyman Christiansen. All rights reserved.
//

#include "thread_helper.h"


thread_helper::thread_helper() {

}

thread_helper::~thread_helper() {

}


void thread_helper::smooth(dsc_obj &dsc, double left, double right) {
    num_thread_ = 8;
    workers_.clear();

    left -= 1;
    right += 1;

    std::vector<DSC2D::vec3> new_pos;
    new_pos.resize(dsc.get_no_vertices());

    // 2. Launch threads
    for (int i = 0; i < num_thread_; i++){
        workers_.push_back(std::thread(smooth_thread, i, this, &dsc, left, right, &new_pos));
    }


    for(auto & th: workers_){
        th.join();
    }

    // 3. Re assign coordinateR
    dsc.swap_pos(&new_pos);
}

void thread_helper::edge_collapse(dsc_obj & dsc, double left, double right){
    /////////////////////////////////////////////////////////////////////////////
    // 1. Determine the region
    num_thread_ = 3;
    workers_.clear();
    
    left -= 1;
    right += 1;
    
    // 2. Lauch threads
    count_finish = 0;
    for (int i = 0; i < num_thread_; i++){
        workers_.push_back(std::thread(edge_colapse_worker, i, this, &dsc, left, right));
    }
    
    
    for(auto & th: workers_){
        th.join();
    }
}

void thread_helper::edge_colapse_worker(int th_index, thread_helper *th_mangr, dsc_obj * dsc,
                         double left, double right){
    
    double witdth = (right - left) / (2. * th_mangr->num_thread_);
    
    // 1. Collapse first group
    collapse_region(left + 2*th_index*witdth, left + (2*th_index + 1)*witdth + witdth, dsc);
    
    // 2. Wait for other threads
    std::unique_lock<std::mutex> lck(th_mangr->mtx_all_);
    th_mangr->count_finish ++;
    th_mangr->cv.notify_all();
    while (th_mangr->count_finish < th_mangr->num_thread_) {
        th_mangr->cv.wait(lck);
    }
    
    
    
    // 3. Collapse second group
    collapse_region(left + (2*th_index + 1)*witdth, left + (2*th_index + 2)*witdth, dsc);
}

void thread_helper::collapse_region(double left, double right, dsc_obj * dsc){
    for(auto ei = dsc->halfedges_begin(); ei != dsc->halfedges_end(); ei++)
    {
        auto p = dsc->get_pos(dsc->walker(*ei).vertex());
        if((p[0] > left && p[0] < right )
           && (p[1] > left && p[1] < right))
        {
          //  if(dsc->mesh->in_use(*ei))
            {
                if(dsc->length(*ei) < dsc->DEG_LENGTH*dsc->AVG_LENGTH && !dsc->collapse(*ei, true)){
                    dsc->collapse(*ei, false);
#ifdef PROFILE_TH
                    
#endif
                }
            }
        }
    }
}

#define ALPHA 1.0

void thread_helper::smooth_thread(int th_index, thread_helper *th_mangr, dsc_obj * dsc,
                            double left, double right, std::vector<DSC2D::vec3> * new_pos) {

    double witdth = (right - left)/(double)th_mangr->num_thread_;
    left = left + th_index * witdth;
    right = left + witdth;
    
    int i = 0;
    for(auto vi = dsc->vertices_begin(); vi != dsc->vertices_end(); ++vi,++i)
    {
        auto p = dsc->get_pos(*vi);
        if(p[0] >= left && p[0] < right){
            if(dsc->safe_editable(*vi))
            {
                p = ALPHA*(dsc->get_barycenter(*vi, false) - dsc->get_pos(*vi)) + dsc->get_pos(*vi);
            }
            new_pos->at(i) = DSC2D::vec3(p[0], p[1], 0.);
        }
    }
}
