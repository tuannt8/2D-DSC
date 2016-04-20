//
//  texture_segment.cpp
//  DSC_texture_seg
//
//  Created by Tuan Nguyen Trung on 4/7/16.
//  Copyright © 2016 Asger Nyman Christiansen. All rights reserved.
//

#include "texture_segment.hpp"
#include "setting_file.h"
#include "options_disp.h"
#include "helper.h"

using namespace std;
#define INT_BIG 99999

texture_segment::texture_segment()
{
    
}

texture_segment::~texture_segment()
{
    
}

void get_bounding_box(const vector<Vec2> & pts , CGLA::Vec2i & ld, CGLA::Vec2i & ru)
{
    ld = CGLA::Vec2i(INT_BIG);
    ru = CGLA::Vec2i(-INT_BIG);
    
    for (auto & p : pts)
    {
        ld[0] = (int)min((double)ld[0], p[0]);
        ld[1] = (int)min((double)ld[1], p[1]);
        ru[0] = (int)max((double)ru[0], p[0]);
        ru[1] = (int)max((double)ru[1], p[1]);
    }
}

bool intersect_tri(const vector<Vec2> & pts, double x, double & miny, double &maxy)
{
    miny = INFINITY;
    maxy = -INFINITY;
    
    static int idx[] = {1, 2, 0};
    
    for (int i = 0; i < 3; i++)
    {
        auto p0 = pts[i];
        auto p1 = pts[idx[i]];
        
        {
            double t = (x+0.1-p0[0]) / (p1[0] - p0[0]);
            if (t >= 0 and t <= 1)
            {
                double y = p0[1] + t*(p1[1] - p0[1]);
                miny = min(miny, y);
                maxy = max(maxy, y);
            }
        }
        {// To make sure that we don't miss any intersection. Optimize later.
            double t = (x+0.9-p0[0]) / (p1[0] - p0[0]);
            if (t >= 0 and t <= 1)
            {
                double y = p0[1] + t*(p1[1] - p0[1]);
                miny = min(miny, y);
                maxy = max(maxy, y);
            }
        }
    }
    
    return miny != INFINITY;
}

void texture_segment::update_probability()
{
    // 1. Update the labeled image
    for (auto im : _labeled_imgs)
    {
        im->fill(0.0);
    }
    
    for(auto tri : _dsc->faces())
    {
        int phase = _dsc->get_label(tri);
        
        auto pts = _dsc->get_pos(tri);
        CGLA::Vec2i ld, ru;
        get_bounding_box(pts, ld, ru);
        
        // The triangle is convect. It is easy
        double min_y, max_y;
        for (int x = floor(ld[0]); x < ceil(ru[0]); x++)
        {
            // Find intersection
            if (intersect_tri(pts, x, min_y, max_y))
            {
                // update dictionary
                for (int y = floor(min_y); y < ceil(max_y); y++)
                {
                    _labeled_imgs[phase]->set_value(x,y,1.0);
                }
            }
        }
    }
    
    // For visualization
    _labeled_over_lay_img->averaging(_labeled_imgs);
    
//    _labeled_imgs[0]->ex_display("Label 0");
//    _labeled_imgs[1]->ex_display("Label 1");
    
    // 2. Update the probability image
    for (int i = 0; i < _labeled_imgs.size(); i++)
    {
        auto l_m = _labeled_imgs[i]->reshape_to_vector();
        auto probability = _dict->compute_probability(l_m);
        
        _probability_imgs[i]->update(probability);
    }
    
    // Normalize the probability
    std::vector<double> area(_probability_imgs.size(), 0.0);
    for (auto tri : _dsc->faces())
    {
        int phase = _dsc->get_label(tri);
        area[phase] += _dsc->area(tri);
    }
    smooth_image::area_normalization(_probability_imgs, area);
    
    
    _probability_over_lay_img->averaging(_probability_imgs);
    
//    _probability_imgs[0]->ex_display("Probability 0");
//    _probability_imgs[1]->ex_display("Probability 1");
    
    
    optimize_label();
}

void texture_segment::draw_test_coord()
{    
    glBegin(GL_TRIANGLES);
    for (auto tri: _dsc->faces())
    {
        for (auto pt : _dsc->get_pos(tri))
        {
            double c = _probability_imgs[0]->get_value_f(pt[0], pt[1]);
            glColor3f(c, c, c);
            glVertex2dv(pt.get());
        }
    }
    glEnd();
}

void texture_segment::draw_dictionary()
{
    if (_labeled_over_lay_img)
    {
        _labeled_over_lay_img->draw_image();
    }
}

void texture_segment::draw_probability()
{
    if(_probability_over_lay_img)
    {
        _probability_over_lay_img->draw_image();
    }
}

#define UPDATE_PROB_FREQUENCY 100
void texture_segment::update_dsc()
{
//    static int count = UPDATE_PROB_FREQUENCY;
//    if (count == UPDATE_PROB_FREQUENCY)
//    {
//        update_probability();
//        count = 0;
//        std::cout << "Probability updated\n";
//    }
//    count ++;
    
    displace_dsc();
    
    // Compute vertex forces
    compute_probability_forces();
    
    compute_curvature_force();
    

}

void texture_segment::optimize_label()
{
    _tri_variation_debug.resize(_dsc->get_no_faces());
    
    for (auto tri : _dsc->faces())
    {
        auto pts = _dsc->get_pos(tri);
        
        // 1. Find highest probability
        std::vector<double> probs;
        for (auto p : _probability_imgs)
        {
            probs.push_back(p->sum_over_tri(pts));
        }
        
        auto max_p = std::max_element(probs.begin(), probs.end());
        int max_phase = (int)(max_p - probs.begin());
        
        // 2. Probability variation
        double mean_p = (*max_p) / _dsc->area(tri);
        auto var = _probability_imgs[max_phase]->get_variation_tri(pts, mean_p);
        
        _tri_variation_debug[tri] = tri_variation({max_phase, var});
        
        if (max_phase == _dsc->get_label(tri))
        {
            continue;
        }
        
        // 3. Relabel of subdivide
        // Relabeling does not need stable, but subdivision need stable.
        if (var < 0.5) // relabel
        {
            _dsc->update_attributes(tri, max_phase);
        }
        

    }
}



void texture_segment::draw_debug()
{
    if (options_disp::get_option("Triangle variation", false))
    {
        helper_t::autoColor colors;
//        static std::vector<Vec3> colors = {Vec3(1,0,0), Vec3(0,1,0), Vec3(0,0,1),
//            Vec3(0,1,1),Vec3(1,0,1), Vec3(1,1,0)};
        
        for (auto tri : _dsc->faces())
        {
            auto pts = _dsc->get_pos(tri);
            auto & variation = _tri_variation_debug[tri];
            glBegin(GL_TRIANGLES);
            glColor3dv(colors[variation.phase].get());
            glVertex2dv(pts[0].get());
            glVertex2dv(pts[1].get());
            glVertex2dv(pts[2].get());
            glEnd();
            
            glColor3f(0, 0, 0);
            auto c = helper_t::get_barry_center(pts);
            std::ostringstream os; os << variation.variation;
            helper_t::gl_text(c[0], c[1], os.str());
        }
    }
}

void texture_segment::compute_probability_forces()
{
    for (auto eid : _dsc->halfedges())
    {
        auto hew = _dsc->walker(eid);
        
        // Only interface edge, and one time per edge
        if (!_dsc->is_interface(eid)
            or hew.halfedge().get_index() > hew.opp().halfedge().get_index())
        {
            continue;
        }
        
        int phase0 = _dsc->get_label(hew.face());
        int phase1 = _dsc->get_label(hew.opp().face());
        
        // Loop on the edge
        auto p0 = _dsc->get_pos(hew.opp().vertex());
        auto p1 = _dsc->get_pos(hew.vertex());
        double length = (p1 - p0).length();
        assert (length > 0.001);
        
        int N = ceil(length);
        double dl = (double)length / (double)N;
        
        double f0 = 0.0, f1 = 0.0; // The two forces
        for (int i = 0; i <= N; i++)
        {
            auto p = p0 + (p1 - p0)*((double)i / (double)N);
            auto prob_0 = _probability_imgs[phase0]->get_value_f(p[0], p[1]);
            auto prob_1 = _probability_imgs[phase1]->get_value_f(p[0], p[1]);
            
            auto f = (prob_0 - prob_1) * dl / length * 10;
            
            f0 += f*(N-i)/(double)N;
            f1 += f*i/(double)N;
        }
        
        // Normal vector
        Vec2 L01 = p1 - p0;
        L01.normalize();
        Vec2 N01(L01[1], -L01[0]);
        
        assert(f0 != NAN and f1 != NAN);
        
        _dsc->add_node_external_force(hew.opp().vertex(), N01*f0);
        _dsc->add_node_external_force(hew.vertex(), N01*f1);
    }
}

void texture_segment::displace_dsc()
{
    for (auto ni = _dsc->vertices_begin(); ni != _dsc->vertices_end(); ni++)
    {
        _dsc->bStable[*ni] = 1;
        
        if ((_dsc->is_interface(*ni) or _dsc->is_crossing(*ni)))
        {
            Vec2 dis = (_dsc->get_node_internal_force(*ni)
                        + _dsc->get_node_external_force(*ni));
            assert(dis.length() != NAN);
            
            double n_dt = 0.1;//s_dsc->time_step(*ni);
            
            _dsc->set_destination(*ni, _dsc->get_pos(*ni) + dis*n_dt);
            
        }
    }
    
    _dsc->deform();
}

void texture_segment::compute_curvature_force()
{
    double alpha = 0.1;
    for (auto vkey : _dsc->vertices())
    {
        if (_dsc->is_interface(vkey)
            or _dsc->is_crossing(vkey))
        {
            for(auto w = _dsc->walker(vkey); !w.full_circle(); w = w.circulate_vertex_ccw())
            {
                if (_dsc->is_interface(w.halfedge()))
                {
                    auto p12 = _dsc->get_pos(w.vertex()) - _dsc->get_pos(w.opp().vertex());
                    assert(p12.length() > 0.001);

                    p12.normalize();
                    _dsc->add_node_internal_force(vkey, p12*alpha);

                }
            }
        }
    }
}


void texture_segment::show_all_probablity()
{
    
    
    cimg_library::CImgList<double> imglist;
    
    for (auto p : _probability_imgs)
    {
        imglist.push_back(p->_core_img);
    }

    imglist.display("Probabilities");
    
}

void texture_segment::init()
{
    // Load image
    _origin_img = std::shared_ptr<smooth_image>(new smooth_image);
    _origin_img->load_image(setting_file._image_name);
    
    // Initialize the probability
    for (int i = 0; i< setting_file._circle_inits.size() + 1; i++)
    {
        _labeled_imgs.push_back(std::shared_ptr<smooth_image>(
                            new smooth_image(_origin_img->width(), _origin_img->height()))
                                );
        _probability_imgs.push_back(std::shared_ptr<smooth_image>
                                (
                                 new smooth_image(_origin_img->width(), _origin_img->height()))
                                );
    }
    _labeled_over_lay_img = std::shared_ptr<smooth_image>(
                        new smooth_image(_origin_img->width(), _origin_img->height())
                                         );
    _probability_over_lay_img = std::shared_ptr<smooth_image>(
                        new smooth_image(_origin_img->width(), _origin_img->height())
                                                          );
    // Construct dictionary
    _dict = std::unique_ptr<texture::dictionary>
                (new texture::dictionary(setting_file._image_name));
    
//    _dict = std::unique_ptr<texture::dictionary>
//                (new texture::dictionary("DATA/randen15.png"));

}

void texture_segment::init_dsc_phases()
{
    // Relabel
    auto circle_init = setting_file._circle_inits;
    for (auto tri : _dsc->faces())
    {
        auto pts = _dsc->get_pos(tri);
        // check if it belong to any circle
        for (int i = 0; i < circle_init.size(); i++)
        {
            auto & circles = circle_init[i];
            for (int j = 0; j < circles.size(); j++)
            {
                auto & circle = circles[j];
                if(circle.is_in_circle(pts)){
                    _dsc->update_attributes(tri, i+1); // Phase 0 is reserved for background
                }
            }
        }
    }
    
    // Deform
    for (auto v:_dsc->vertices())
    {
        if (_dsc->is_interface(v))
        {
            auto pt = _dsc->get_pos(v);
            // check if it belong to any circle
            for (int i = 0; i < circle_init.size(); i++)
            {
                auto & circles = circle_init[i];
                for (int j = 0; j < circles.size(); j++)
                {
                    auto & circle = circles[j];
                    if(circle.is_in_circle(pt)){
                        auto pc = circle.project_to_circle(pt);
                        _dsc->set_destination(v, pc);
                    }
                }
            }
        }
    }
    _dsc->deform();
}