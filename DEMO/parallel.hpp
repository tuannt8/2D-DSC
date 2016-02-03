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
#include <mutex>
#include <boost/thread/locks.hpp>
#include <boost/thread/shared_mutex.hpp>

#define NUM_THREADS 8
#define CHUNK_SIZE 20

typedef boost::shared_mutex Lock;
typedef boost::unique_lock< Lock >  WriteLock;
typedef boost::shared_lock< Lock >  ReadLock;

class parallel
{
public:
    typedef DSC2D::DeformableSimplicialComplex dsc_class;

    
public:
    parallel(){};
    ~parallel(){};
    
    
    void random_flip(dsc_class &dsc, double proba = 0.2);
    
public:

    void parallel_flip_edge(dsc_class &dsc);
};


#endif /* parallel_hpp */
