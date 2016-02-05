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
#include <functional>

#define NUM_THREADS 8
#define CHUNK_SIZE 100


#define _min(a,b) (a<b?a:b)


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

class parallel
{
public:
    typedef DSC2D::DeformableSimplicialComplex dsc_class;

    
public:
    parallel(){};
    ~parallel(){};
    
    static std::vector<HMesh::FaceID> one_ring_triangle(parallel::dsc_class *dsc, HMesh::FaceID fid);
    static std::vector<HMesh::HalfEdgeID> neighbor_edge(parallel::dsc_class * dsc, HMesh::HalfEdgeID eid);
    
    // Parallel edges
    static void parallel_thread_edges(dsc_class &dsc, std::function<void(dsc_class *, std::vector<HMesh::HalfEdgeID> *, int, int, Barrier&)> func);
    
    // Parallel faces
    static void parallel_thread_faces(dsc_class &dsc, std::function<void(dsc_class *, std::vector<HMesh::FaceID> *, int, int, Barrier&)> func);
    
    // Parallel vertices
    static void parallel_thread_vertices(dsc_class &dsc, std::function<void(dsc_class *, std::vector<HMesh::VertexID> *, int, int, Barrier&)> func);
public:
    // Edge flip
    void random_flip(dsc_class &dsc, double proba = 0.1);
    void parallel_flip_edge(dsc_class &dsc);
    void serial_flip(dsc_class *dsc);
    
    // Remove degenerate edges
    void random_irregular_edge(dsc_class &dsc, double proba = 0.2);
    void parallel_remove_degenerate_edges(dsc_class &dsc);
    void serial_remove_degenerate_edges(dsc_class *dsc);
    
    // Remove degenerate face
    void parallel_remove_degenerate_faces(dsc_class &dsc);
    void random_shrink_face(dsc_class &dsc, double proba = 0.2);
    void serial_remove_degenerate_faces(dsc_class *dsc);
    
    // Smooth
    void parallel_smooth(dsc_class &dsc);
};


class time_profile
{
public:
    time_profile(std::string text = "Process")
    {
        start = std::chrono::system_clock::now();
        m_text = text;
    }
    
    ~time_profile()
    {
        std::chrono::duration<double> t = std::chrono::system_clock::now() - start;
        printf("-- %s in time: %f --\n", m_text.c_str(), t.count());
    }
    
    std::chrono::system_clock::time_point start;
    std::string m_text;
};


#endif /* parallel_hpp */
