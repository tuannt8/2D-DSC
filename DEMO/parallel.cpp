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

//
//void flip_worker3(parallel::dsc_class *dsc, std::vector<HMesh::HalfEdgeID> * all_edges, int start, int stop,
//                  Barrier& cur_barier)
//{
//    
//    HMesh::MinAngleEnergy energy_fun(dsc->MIN_ANGLE);
//    // Find potential edge first
//    std::vector<HMesh::HalfEdgeID> to_flip;
//    for (int i = start; i < stop; i++)
//    {
//        auto eid = all_edges->at(i);
//        auto hew = dsc->walker(eid);
//        if (hew.halfedge().get_index() > hew.opp().halfedge().get_index())
//        {
//            continue;
//        }
//        
//        double ene = energy_fun.delta_energy(*dsc->mesh, eid);
//        if (ene < 0
//            && HMesh::precond_flip_edge(*dsc->mesh, eid)
//            && dsc->safe_editable(eid))
//        {
//            if(HMesh::precond_flip_edge(*dsc->mesh, eid))
//                to_flip.push_back(eid);
//        }
//    }
//    
//    cur_barier.Wait();
//    
//    // Perform flip
//    int count = 0;
//    for (auto e:to_flip)
//    {
//        dsc_stdlock.lock();
//        if (!invalid[e.get_index()]
//            && dsc->mesh->in_use(e))
//        {
//            // Invalid the neighbor
//            auto hew = dsc->walker(e);
//            invalid[hew.next().halfedge().get_index()] = 1;
//            invalid[hew.next().next().halfedge().get_index()] = 1;
//            invalid[hew.opp().next().halfedge().get_index()] = 1;
//            invalid[hew.opp().next().next().halfedge().get_index()] = 1;
//            
//            dsc_stdlock.unlock();
//            
//            
//            dsc->mesh->flip_edge(e);
//            count ++;
//        }
//        else
//            dsc_stdlock.unlock();
//    }
//}

void parallel::parallel_flip_edge(dsc_class &dsc)
{
    
    auto worker = [](dsc_class *dsc, std::vector<HMesh::HalfEdgeID> * all_edges, int start, int stop,
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
            if (ene < 0
                && HMesh::precond_flip_edge(*dsc->mesh, eid)
                && dsc->safe_editable(eid))
            {
                if(HMesh::precond_flip_edge(*dsc->mesh, eid))
                    to_flip.push_back(eid);
            }
        }
        
        cur_barier.Wait();
        
        // Perform flip
        int count = 0;
        for (auto e:to_flip)
        {
            dsc_stdlock.lock();
            if (!invalid[e.get_index()]
                && dsc->mesh->in_use(e))
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
    };
    
    
    parallel_thread_edges(dsc, worker);
    
    return;
    
//    using namespace std;
//    
//    invalid.reset();
//    
//    
//    std::vector<HMesh::HalfEdgeID> all_edges;
//    for (auto eid = dsc.halfedges_begin(); eid != dsc.halfedges_end(); eid++)
//    {
//        all_edges.push_back(*eid);
//    }
//    
//    int size = (int)(all_edges.size() + NUM_THREADS) / NUM_THREADS;
//
//    // Launch threads
//    std::thread th[NUM_THREADS];
//    Barrier bar(NUM_THREADS);
//    for (int i = 0; i < NUM_THREADS; i++)
//    {
//        th[i] = std::thread(flip_worker3, &dsc, &all_edges, i*size, _min(i*size+size, all_edges.size()),
//                            std::ref(bar));
//    }
//    
//    for (int i = 0; i < NUM_THREADS; i++)
//    {
//        th[i].join();
//    }
}

#define move(a,b) a = a + (b - a)*0.9
void parallel::random_shrink_face(dsc_class &dsc, double proba)
{
    for (auto f : dsc.faces())
    {
        double r = (double)rand() / RAND_MAX;
        if (r < proba)
        {
            auto vs = dsc.get_pos(f);
            auto verts = dsc.get_verts(f);
            auto center = (vs[0] + vs[1] + vs[2]) / 3.0;
            
            move(vs[0], center);
            move(vs[1], center);
            move(vs[2], center);
            
            dsc.set_pos(verts[0], vs[0]);
            dsc.set_pos(verts[1], vs[1]);
            dsc.set_pos(verts[2], vs[2]);
        }
    }
}

void parallel::serial_remove_degenerate_faces(dsc_class *dsc)
{
    time_profile t("Remove face serial");
    
    dsc->remove_degenerate_faces();
}

void parallel:: random_irregular_edge(dsc_class &dsc, double proba)
{
    double degen_length = dsc.DEG_LENGTH * dsc.AVG_LENGTH;
    for (auto eid = dsc.halfedges_begin(); eid != dsc.halfedges_end(); eid++)
    {
        auto hew = dsc.walker(*eid);
//        if (hew.halfedge().get_index() > hew.opp().halfedge().get_index())
//        {
//            continue;
//        }
        
        double r = (double)rand() / RAND_MAX;
        if (r < proba && dsc.length(*eid) > degen_length)
        {
            // Move the vertex
            auto v1 = hew.vertex();
            auto v2 = hew.opp().vertex();
            
            auto p1 = dsc.get_pos(v1);
            auto p2 = dsc.get_pos(v2);
            auto l = (p2 - p1);
            //l.normalize();
            auto newP = p1 + l*(1 - dsc.DEG_LENGTH/2.0);
            
            dsc.set_pos(v1, newP);
        }
    }
}

std::vector<HMesh::HalfEdgeID> neighbor_edge(parallel::dsc_class * dsc, HMesh::HalfEdgeID eid)
{
    std::vector<HMesh::HalfEdgeID> eids;
    auto hew = dsc->walker(eid);

    for (auto inithe = dsc->walker(hew.vertex()); !inithe.full_circle(); inithe = inithe.circulate_vertex_cw())
    {
        eids.push_back(inithe.halfedge());
        eids.push_back(inithe.opp().halfedge());
    }
    
    for (auto inithe = dsc->walker(hew.opp().vertex()); !inithe.full_circle(); inithe = inithe.circulate_vertex_cw())
    {
        eids.push_back(inithe.halfedge());
        eids.push_back(inithe.opp().halfedge());
    }

    return eids;
}

void collapse_worker(parallel::dsc_class *dsc, std::vector<HMesh::HalfEdgeID> * all_edges, int start, int stop,
                  Barrier& cur_barier, int tid)
{
    // Find potential edge first
    std::vector<HMesh::HalfEdgeID> to_collapse;
    std::vector<std::vector<HMesh::HalfEdgeID>> neighbor;
    for (int i = start; i < stop; i++)
    {
        auto eid = all_edges->at(i);
        auto hew = dsc->walker(eid);
        
        if (hew.halfedge().get_index() > hew.opp().halfedge().get_index())
        {
            continue;
        }

        if (dsc->length(eid) < dsc->DEG_LENGTH * dsc->AVG_LENGTH)
        {
            if(HMesh::precond_collapse_edge(*dsc->mesh, eid)
               && dsc->unsafe_editable(eid))
            {
                to_collapse.push_back(eid);
                neighbor.push_back(neighbor_edge(dsc, eid));
            }
        }
    }
    
    cur_barier.Wait();
    
    // Perform flip
    int count = 0;
    for (int i = 0; i < to_collapse.size(); i++)
    {
        auto e = to_collapse[i];
        
        dsc_stdlock.lock();
        if (!invalid[e.get_index()]
            && dsc->mesh->in_use(e))
        {
            // Invalid the neighbor
            for (auto &e : neighbor[i])
            {
                invalid[e.get_index()] = 1;
            }
            
            dsc_stdlock.unlock();
            
            if(!dsc->collapse(e, true))
            {
                dsc->collapse(e, false);
            }
            count ++;
            
            
        }
        else
            dsc_stdlock.unlock();
    }
    
//    printf("%d edges is collapsed \n", count);
}



void parallel::serial_remove_degenerate_edges(dsc_class *dsc)
{
    time_profile t("Remove edge parallel");
    int count;
    dsc->remove_degenerate_edges(&count);
    printf("Serial: %d edges is collapsed \n", count);
}

std::vector<HMesh::FaceID> one_ring_triangle(parallel::dsc_class *dsc, HMesh::FaceID fid)
{
    std::vector<HMesh::FaceID> fids;
    auto vids = dsc->get_verts(fid);
    for (auto vid : vids)
    {
        for (auto hew = dsc->walker(vid); !hew.full_circle(); hew = hew.circulate_vertex_cw())
        {
            if (hew.face() != HMesh::InvalidFaceID)
            {
                fids.push_back(hew.face());
            }
        }
    }
    return fids;
}

void collapse_face_worker(parallel::dsc_class *dsc, std::vector<HMesh::FaceID> * all_faces, int start, int stop, Barrier& cur_barier, int tid)
{
    std::vector<HMesh::FaceID> to_collapse;
    std::vector<std::vector<HMesh::FaceID>> neighbor_faces;
    
    for (int i = start; i < stop; i++)
    {
        HMesh::FaceID fid = all_faces->at(i);
        if (dsc->mesh->in_use(fid)
            and ( dsc->min_angle(fid) < dsc->DEG_ANGLE or dsc->area(fid) < dsc->DEG_AREA*dsc->AVG_AREA))
        {
            to_collapse.push_back(fid);
            // All neighbor
            neighbor_faces.push_back(one_ring_triangle(dsc, fid));
        }
    }
    
    cur_barier.Wait();
    
    for (int i = 0; i < to_collapse.size(); i++ )
    {
        auto f = to_collapse[i];
        
        dsc_stdlock.lock();
        if (!invalid[f.get_index()]
            && dsc->mesh->in_use(f))
        {
            // invalid all neighbor
            auto & tids = neighbor_faces[i];
            for (auto & tid : tids)
            {
                invalid[tid.get_index()] = 1;
            }
            dsc_stdlock.unlock();
            
            // collapse
            if (!dsc->collapse(f, true))
            {
                dsc->collapse(f, false);
            }
        }
        else
            dsc_stdlock.unlock();
    }
}

void parallel::parallel_thread_edges(dsc_class &dsc,
    std::function<void(dsc_class *, std::vector<HMesh::HalfEdgeID> *, int, int, Barrier&) >  func1)
{
    invalid.reset();
    
    
    std::vector<HMesh::HalfEdgeID> all_edges;
    for (auto fid : dsc.halfedges())
    {
        all_edges.push_back(fid);
    }
    
    int size = (int)(all_edges.size() + NUM_THREADS) / NUM_THREADS;
    
    // Launch threads
    std::thread th[NUM_THREADS];
    Barrier bar(NUM_THREADS);
    
    
    for (int i = 0; i < NUM_THREADS; i++)
    {
        th[i] = std::thread(func1, &dsc, & all_edges, i*size, _min(i*size+size, all_edges.size()),
                            std::ref(bar) );
    }
    
    for (int i = 0; i < NUM_THREADS; i++)
    {
        th[i].join();
    }
}

void parallel::parallel_remove_degenerate_faces(dsc_class &dsc)
{
//    time_profile t("remove face parallel");
    
    using namespace std;
    
    invalid.reset();
    
    
    std::vector<HMesh::FaceID> all_edges;
    for (auto fid : dsc.faces())
    {
        all_edges.push_back(fid);
    }
    
    int size = (int)(all_edges.size() + NUM_THREADS) / NUM_THREADS;
    
    // Launch threads
    std::thread th[NUM_THREADS];
    Barrier bar(NUM_THREADS);
    
    
    for (int i = 0; i < NUM_THREADS; i++)
    {
        th[i] = std::thread(collapse_face_worker, &dsc, &all_edges, i*size, _min(i*size+size, all_edges.size()), std::ref(bar), i);
    }
    
    for (int i = 0; i < NUM_THREADS; i++)
    {
        th[i].join();
    }
    
    
//    int i;
//    for (i = 0; i < NUM_THREADS-1; i++)
//    {
//        th[i] = std::thread(collapse_face_worker, &dsc, &all_edges, i*size, _min(i*size+size, all_edges.size()), std::ref(bar), i);
//    }
//    
//    collapse_face_worker(&dsc, &all_edges, i*size, _min(i*size+size, all_edges.size()), std::ref(bar), i);
//    
//    for (int i = 0; i < NUM_THREADS-1; i++)
//    {
//        th[i].join();
//    }
}

void smooth_worker(parallel::dsc_class *dsc, std::vector<HMesh::VertexID> * all_verts, int start, int stop, Barrier& cur_barier, int tid)
{
    std::vector<DSC2D::vec2> positions(stop - start);
    for (int i = start; i < stop; i++)
    {
        auto vid = all_verts->at(i);
        
        if(dsc->safe_editable(vid))
        {
            positions[i - start] = dsc->get_barycenter(vid, false);
        }
    }
    
    cur_barier.Wait();
    
    for (int i = start; i < stop; i++)
    {
        auto vid = all_verts->at(i);
        if(dsc->safe_editable(vid))
        {
            dsc->set_pos(vid, positions[i - start]);
        }
    }
}

void parallel::parallel_smooth(dsc_class &dsc)
{
    std::vector<HMesh::VertexID> all_vertices;
    for (auto vid : dsc.vertices())
    {
        all_vertices.push_back(vid);
    }
    
    int size = (int)(all_vertices.size() + NUM_THREADS) / NUM_THREADS;
    
    // Launch threads
    std::thread th[NUM_THREADS];
    Barrier bar(NUM_THREADS);
    for (int i = 0; i < NUM_THREADS; i++)
    {
        th[i] = std::thread(smooth_worker, &dsc, &all_vertices, i*size, _min(i*size+size, all_vertices.size()), std::ref(bar), i);
    }
    
    for (int i = 0; i < NUM_THREADS; i++)
    {
        th[i].join();
    }
}

void parallel::parallel_remove_degenerate_edges(dsc_class &dsc)
{
//    time_profile t;
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
        th[i] = std::thread(collapse_worker, &dsc, &all_edges, i*size, _min(i*size+size, all_edges.size()), std::ref(bar), i);
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
    
    
    std::chrono::duration<double> t = std::chrono::system_clock::now() - init_time;
    
    printf("--------- Serial in %f sec ----------- \n", t.count());
}














