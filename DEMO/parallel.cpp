//
//  parallel.cpp
//  DSC
//
//  Created by Tuan Nguyen Trung on 2/3/16.
//  Copyright © 2016 Asger Nyman Christiansen. All rights reserved.
//

#include "parallel.hpp"
#include <boost/thread.hpp>

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
std::bitset<100000> invalid;

void flip_worker(parallel::dsc_class *dsc, int tid)
{
    printf("Thread %d launched\n", tid);
    
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
//            // Check if we should flip
//            double r = (double)rand()/RAND_MAX;
//            if (r < 0.1)
//            {
//                l.lock();
//                auto hew = dsc->walker(*eid);
//                auto v1 = dsc->get_pos(hew.vertex());
//                auto v2 = dsc->get_pos(hew.opp().vertex());
//                auto v3 = dsc->get_pos(hew.next().vertex());
//                auto v4 = dsc->get_pos(hew.opp().next().vertex());
//                l.unlock();
//                
//                bool nonOverlap = DSC2D::Util::signed_area(v3, v4, v1) > 0
//                and DSC2D::Util::signed_area(v3, v2, v4) > 0;
//                
//                if (nonOverlap and !invalid[eid->get_index()])
//                {
//                    WriteLock wl(m_dsc);
//                    // Invalid the neighbor
//                    invalid[hew.next().halfedge().get_index()] = 1;
//                    invalid[hew.next().next().halfedge().get_index()] = 1;
//                    invalid[hew.opp().next().halfedge().get_index()] = 1;
//                    invalid[hew.opp().next().next().halfedge().get_index()] = 1;
//                    dsc->mesh->flip_edge(*eid);
//                }
//            }
            
            l.lock();
            auto hew = dsc->walker(*eid);
            double  energy = energy_fun.delta_energy(*dsc->mesh, hew.halfedge());
            l.unlock();
            
            if (energy < 0 and !invalid[eid->get_index()])
            {
                l.lock();
                bool bflip = HMesh::precond_collapse_edge(*dsc->mesh, hew.halfedge())
                                    && dsc->safe_editable(hew.halfedge());
                l.unlock();
                
                if (bflip)
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
    
//    WriteLock wl(m_dsc);
//    printf("T %d: ", tid);
//    for (auto e : eids)
//    {
//        printf("%zu ", e.get_index());
//    }
//    printf("\n");
}

void parallel::parallel_flip_edge(dsc_class &dsc)
{
    using namespace std;
    
    invalid.reset();
    
    // Launch threads
    boost::thread th[NUM_THREADS];
    
    for (int i = 0; i < NUM_THREADS; i++)
    {
        th[i] = boost::thread(flip_worker, &dsc, i);
    }
    
    for (int i = 0; i < NUM_THREADS; i++)
    {
        th[i].join();
    }
    
    printf("--------- All thread done!!!!!! ----------- \n");
}