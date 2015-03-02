//
// Created by Tuan Nguyen Trung on 2/27/15.
// Copyright (c) 2015 Asger Nyman Christiansen. All rights reserved.
//


#ifndef __thread_helper_H_
#define __thread_helper_H_

#include "define.h"
#include <thread>

enum thread_state{
    STATE_NONE
    ,STATE_SMOOTH
    , STATE_END
};

class thread_helper {
public:
    thread_helper();
    ~thread_helper();

    void smooth (dsc_obj & dsc, double l, double r);
    void edge_collapse(dsc_obj & dsc);

public:
    // Thread params
    std::mutex mtx_all_;
    std::condition_variable cv;
    int count_finish;
    int count_finish_2;
    
    int num_thread_;
    std::vector<std::thread> workers_;

    static void smooth_thread(int th_index, thread_helper *th_mangr, dsc_obj * dsc,
                        double left, double right, std::vector<DSC2D::vec3> *new_pos);
    

    static void edge_colapse_worker(int th_index, thread_helper *th_mangr, dsc_obj * dsc,
                                    double left, double right);
    static void collapse_region(double left, double right, dsc_obj * dsc);
    
public: // debug
    int count_change = 0;
};

#endif //__thread_helper_H_
