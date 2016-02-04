//
//  parallel.cpp
//  DSC
//
//  Created by Tuan Nguyen Trung on 2/3/16.
//  Copyright © 2016 Tuan Nguyen Trung. All rights reserved.
//

#include "parallel.hpp"
#include <mutex>
#include <thread>

std::bitset<10000000> invalid;
std::mutex dsc_stdlock;

void parallel::random_flip(dsc_class &dsc, double proba)
{
    for (int i = 0; i < 10; i++)
    {
        for (auto eid = dsc.halfedges_begin(); eid != dsc.halfedges_end(); eid++)
        {
            auto hew = dsc.walker(*eid);
            if (hew.halfedge().get_index() > hew.opp().halfedge().get_index())
            {
                continue;
            }
            
            double r = (double)rand() / RAND_MAX;
            if (r < proba
                and dsc.safe_editable(*eid)
                and HMesh::precond_collapse_edge(*dsc.mesh, *eid))
            {
                auto v1 = dsc.get_pos(hew.vertex());
                auto v2 = dsc.get_pos(hew.opp().vertex());
                auto v3 = dsc.get_pos(hew.next().vertex());
                auto v4 = dsc.get_pos(hew.opp().next().vertex());
                
                bool nonOverlap = DSC2D::Util::signed_area(v3, v4, v1) > 0
                                and DSC2D::Util::signed_area(v3, v2, v4) > 0;
                
                if (nonOverlap)
                {
                    dsc.mesh->flip_edge(*eid);
                }
                
            }
        }
    }
}


void flip_worker3(parallel::dsc_class *dsc, std::vector<HMesh::HalfEdgeID> * all_edges, int start, int stop,
                  Barrier& cur_barier)
{
    
    HMesh::MinAngleEnergy energy_fun(dsc->MIN_ANGLE);
    // Find potential edge first
    std::vector<HMesh::HalfEdgeID> to_flip;
    for (int i = start; i < stop; i++)
    {
        auto eid = all_edges->at(i);
        auto hew = dsc->walker(eid);
        if (hew.halfedge().get_index() > hew.opp().halfedge().get_index())
        {
            continue;
        }
        
        double ene = energy_fun.delta_energy(*dsc->mesh, eid);
        if (ene < 0)
        {
            if(HMesh::precond_collapse_edge(*dsc->mesh, eid))
                to_flip.push_back(eid);
        }
    }
    
    cur_barier.Wait();
    
    // Perform flip
    int count = 0;
    for (auto e:to_flip)
    {
        dsc_stdlock.lock();
        auto valid = !invalid[e.get_index()];
        
        
        if (valid)
        {
            // Invalid the neighbor
            auto hew = dsc->walker(e);
            invalid[hew.next().halfedge().get_index()] = 1;
            invalid[hew.next().next().halfedge().get_index()] = 1;
            invalid[hew.opp().next().halfedge().get_index()] = 1;
            invalid[hew.opp().next().next().halfedge().get_index()] = 1;
            
            dsc_stdlock.unlock();
            
            
            dsc->mesh->flip_edge(e);
            count ++;
        }
        else
            dsc_stdlock.unlock();
    }
}

void parallel::parallel_flip_edge(dsc_class &dsc)
{
    using namespace std;
    
    invalid.reset();
    
    
    std::vector<HMesh::HalfEdgeID> all_edges;
    for (auto eid = dsc.halfedges_begin(); eid != dsc.halfedges_end(); eid++)
    {
        all_edges.push_back(*eid);
    }
    
    int size = (int)(all_edges.size() + NUM_THREADS) / NUM_THREADS;

    // Launch threads
    std::thread th[NUM_THREADS];
    Barrier bar(NUM_THREADS);
    for (int i = 0; i < NUM_THREADS; i++)
    {
        th[i] = std::thread(flip_worker3, &dsc, &all_edges, i*size, _min(i*size+size, all_edges.size()),
                            std::ref(bar));
    }
    
    for (int i = 0; i < NUM_THREADS; i++)
    {
        th[i].join();
    }
}

void parallel::serial_flip(dsc_class * dsc)
{
    auto init_time = std::chrono::system_clock::now();
    
    dsc->max_min_angle();
    


    int count = 0;
//    HMesh::MinAngleEnergy energy_fun(dsc->MIN_ANGLE);
//    for (auto eid = dsc->halfedges_begin(); eid != dsc->halfedges_end(); eid++)
//    {
//        auto hew = dsc->walker(*eid);
//        
//        if (hew.halfedge().get_index() > hew.opp().halfedge().get_index())
//        {
//            continue;
//        }
//        
//        double  energy = energy_fun.delta_energy(*dsc->mesh, hew.halfedge());
//
//        
//        if (energy < 0
//   //         && HMesh::precond_collapse_edge(*dsc->mesh, hew.halfedge())
//   //         &&  dsc->safe_editable(hew.halfedge())
//            )
//        {
//            dsc->mesh->flip_edge(*eid);
//            count ++;
//        }
//    }
    
    std::chrono::duration<double> t = std::chrono::system_clock::now() - init_time;
    
    printf("--------- Serial of %d edges in %f sec ----------- \n", count, t.count());
}














