//
//  parallel.hpp
//  DSC
//
//  Created by Tuan Nguyen Trung on 2/3/16.
//  Copyright © 2016 Asger Nyman Christiansen. All rights reserved.
//

#ifndef parallel_hpp
#define parallel_hpp

#include <stdio.h>
#include "DSC.h"

#define NUM_THREADS 6
#define CHUNK_SIZE 100


#define _min(a,b) (a<b?a:b)

class parallel
{
public:
    typedef DSC2D::DeformableSimplicialComplex dsc_class;

    
public:
    parallel(){};
    ~parallel(){};
    
    
    void random_flip(dsc_class &dsc, double proba = 0.01);
    
public:
    // Split work outside and avoid lock
    void parallel_flip_edge(dsc_class &dsc);

    void serial_flip(dsc_class *dsc);
    
};

class Barrier
{
private:
    std::mutex _mutex;
    std::condition_variable _cv;
    std::size_t _count;
public:
    explicit Barrier(std::size_t count) : _count{count} { }
    void Wait()
    {
        std::unique_lock<std::mutex> lock{_mutex};
        if (--_count == 0) {
            _cv.notify_all();
        } else {
            _cv.wait(lock, [this] { return _count == 0; });
        }
    }
};


#endif /* parallel_hpp */
