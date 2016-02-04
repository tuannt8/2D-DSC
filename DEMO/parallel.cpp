//
//  parallel.cpp
//  DSC
//
//  Created by Tuan Nguyen Trung on 2/3/16.
//  Copyright © 2016 Tuan Nguyen Trung. All rights reserved.
//

#include "parallel.hpp"
#include <boost/thread.hpp>
#include <boost/thread/barrier.hpp>
#include <boost/bind.hpp>
#include <boost/atomic.hpp>
#include <mutex>
#include <thread>



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


Lock m_dsc;
std::bitset<10000000> invalid;
std::vector<HMesh::HalfEdgeID> e_ts[NUM_THREADS];

void flip_worker(parallel::dsc_class *dsc, int tid)
{
    int idx = 0;

    ReadLock l(m_dsc);
    auto eid = dsc->halfedges_begin();
    HMesh::MinAngleEnergy energy_fun(dsc->MIN_ANGLE);
    l.unlock();
    

    while (1)
    {
        int idIter = (idx % (NUM_THREADS*CHUNK_SIZE))/CHUNK_SIZE;
        if (idIter == tid
            and !invalid[eid->get_index()])
        {
            
            l.lock();
            auto hew = dsc->walker(*eid);
            double  energy = energy_fun.delta_energy(*dsc->mesh, hew.halfedge());
            l.unlock();
            
            if (energy < 0 and !invalid[eid->get_index()])
            {
                WriteLock wl(m_dsc);
                // Invalid the neighbor
                invalid[hew.next().halfedge().get_index()] = 1;
                invalid[hew.next().next().halfedge().get_index()] = 1;
                invalid[hew.opp().next().halfedge().get_index()] = 1;
                invalid[hew.opp().next().next().halfedge().get_index()] = 1;
                dsc->mesh->flip_edge(*eid);
            }
        }
        
        eid++;
        idx++;
        
        l.lock();
        auto leid = dsc->halfedges_end();
        l.unlock();
        
        if (eid == leid)
        {
            break;
        }
    }
}

void parallel::parallel_flip_edge(dsc_class &dsc)
{
    auto init_time = std::chrono::system_clock::now();
    // dsc.max_min_angle();
    // serial_flip(&dsc);
    parallel_flip_edge3(dsc);

    
//    using namespace std;
//    invalid.reset();
//    // Launch threads
//    boost::thread th[NUM_THREADS];
//    
//    for (int i = 0; i < NUM_THREADS; i++)
//    {
//        th[i] = boost::thread(flip_worker, &dsc, i);
//    }
//    
//    for (int i = 0; i < NUM_THREADS; i++)
//    {
//        th[i].join();
//    }

    std::chrono::duration<double> t = std::chrono::system_clock::now() - init_time;
    printf("--------- All thread done in %f sec ----------- \n", t.count());
}

void flip_worker2(parallel::dsc_class *dsc, std::vector<HMesh::HalfEdgeID> * all_edges, int start, int stop)
{
    
    ReadLock l(m_dsc);
    l.unlock();
    
    HMesh::MinAngleEnergy energy_fun(dsc->MIN_ANGLE);
    
    for (int i = start; i < stop; i++)
    {
        auto eid = all_edges->at(i);
        
        if (!invalid[eid.get_index()])
        {
      //      l.lock();
            
            ReadLock l(m_dsc);
            auto hew = dsc->walker(eid);
            double  energy = energy_fun.delta_energy(*dsc->mesh, hew.halfedge());
            l.unlock();
            
            
            if (energy < 0 and !invalid[eid.get_index()])
            {
                {
                    WriteLock wl(m_dsc);
                    // Invalid the neighbor
                    dsc->mesh->flip_edge(eid);
                    invalid[hew.next().halfedge().get_index()] = 1;
                    invalid[hew.next().next().halfedge().get_index()] = 1;
                    invalid[hew.opp().next().halfedge().get_index()] = 1;
                    invalid[hew.opp().next().next().halfedge().get_index()] = 1;
                }

            }
        }
    }
}

#define _min(a,b) (a<b?a:b)

void parallel::parallel_flip_edge2(dsc_class &dsc)
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
    boost::thread th[NUM_THREADS];
    for (int i = 0; i < NUM_THREADS; i++)
    {
        th[i] = boost::thread(flip_worker2, &dsc, &all_edges, i*size, _min(i*size+size, all_edges.size()));
    }
    
    for (int i = 0; i < NUM_THREADS; i++)
    {
        th[i].join();
    }
    

}

std::mutex dsc_stdlock;
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
       //     dsc->mesh->flip_edge(eid);
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
            auto hew = dsc->walker(e);

            {
                // Invalid the neighbor
            //    dsc_stdlock.lock();
                invalid[hew.next().halfedge().get_index()] = 1;
                invalid[hew.next().next().halfedge().get_index()] = 1;
                invalid[hew.opp().next().halfedge().get_index()] = 1;
                invalid[hew.opp().next().next().halfedge().get_index()] = 1;
                dsc_stdlock.unlock();
            }
            
            
            dsc->mesh->flip_edge(e);
            count ++;
        }
        else
            dsc_stdlock.unlock();
    }
}

void parallel::parallel_flip_edge3(dsc_class &dsc)
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
//    boost::barrier bar(NUM_THREADS);
//    boost::atomic<int> current(0);
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














