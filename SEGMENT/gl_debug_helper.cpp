//
//  gl_debug_helper.cpp
//  DSC
//
//  Created by Tuan Nguyen Trung on 2/17/15.
//  Copyright (c) 2015 Asger Nyman Christiansen. All rights reserved.
//

#include "gl_debug_helper.h"
#ifdef WIN32
#include <GL/glew.h>
#include <GL/glut.h>
#include <GLGraphics/SOIL.h>
#else
#include <GEL/GL/glew.h>
#include <GLUT/glut.h>
#include <GEL/GLGraphics/SOIL.h>
#endif


gl_debug_helper & gl_debug_helper::get_instance(){
    static gl_debug_helper instance;
    return instance;
}
void gl_debug_helper::change_state(){
    get_instance().active_ = ! get_instance().active_;
}
void gl_debug_helper::begin(){
    get_instance().active_ = true;
}
void gl_debug_helper::end(){
    get_instance().active_ = false;
}

void gl_debug_helper::mouseMove(int x, int y){
    if (get_instance().drawing_) {
        get_instance().right_up_ = get_instance().to_gl_coord(Vec2(x, y));
    }
}

void gl_debug_helper::mouseDown(int button, int state, int x, int y){
    if (get_instance().active_) {
        if (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN) {
            get_instance().drawing_ = true;
            get_instance().left_down_ = get_instance().to_gl_coord(Vec2(x, y));
            get_instance().right_up_ = get_instance().to_gl_coord(Vec2(x, y));
        }
        else if (button == GLUT_LEFT_BUTTON && state == GLUT_UP){
            get_instance().drawing_ = false;
            get_instance().right_up_ = get_instance().to_gl_coord(Vec2(x, y));
        }
    }
}

Vec2 gl_debug_helper::to_gl_coord(Vec2 win_coord){
    Vec2 gl_coord;
    
    gl_coord[0] = (win_coord[0] - window_draw_left_[0])*scale_gl_over_win_[0];
    gl_coord[1] = (WIN_HEIGHT_ - win_coord[1] - window_draw_left_[1])*scale_gl_over_win_[1];
    
    return gl_coord;
}

void gl_debug_helper::coord_transform(Vec2 window_draw_left, Vec2 scale, double win_height){
    get_instance().window_draw_left_ = window_draw_left;
    get_instance().scale_gl_over_win_ = scale;
    get_instance().WIN_HEIGHT_ = win_height;
}

void gl_debug_helper::draw(){
    if (get_instance().active_) {
        Vec2 ld = get_instance().left_down_;
        Vec2 ru = get_instance().right_up_;
        
        static Vec2 corner[4];
        corner[0] = ld;
        corner[1] = Vec2(ru[0], ld[1]);
        corner[2] = ru;
        corner[3] = Vec2(ld[0], ru[1]);
        
        glColor3f(1.0, 0.3, 0.2);
        glBegin(GL_LINES);
        for (int i = 0; i < 4; i++) {
            glVertex2dv(corner[i].get());
            glVertex2dv(corner[(i+1)%4].get());
        }
        glEnd();
    }
}
namespace temp_gdh {

    inline bool is_greater(Vec2 g, Vec2 l){
        return (g[0] > l[0]) && (g[1] > l[1]);
    }

    inline Vec2 left_down(Vec2 p1, Vec2 p2){
        Vec2 p(INFINITY, INFINITY);
        p[0] = std::min(p1[0], p2[0]);
        p[1] = std::min(p1[1], p2[1]);
        return p;
    }

    inline Vec2 right_up(Vec2 p1, Vec2 p2){
        Vec2 p(INFINITY, INFINITY);
        p[0] = std::max(p1[0], p2[0]);
        p[1] = std::max(p1[1], p2[1]);
        return p;
    }
}

void gl_debug_helper::print_debug_info(dsc_obj &complex){
    Vec2 pt1 = get_instance().left_down_;
    Vec2 pt2 = get_instance().right_up_;

    Vec2 ld = temp_gdh::left_down(pt1, pt2);
    Vec2 ru = temp_gdh::right_up(pt1, pt2);
    
    if (LOG_VERTEX) {
        std::cout << "Node keys: ";
        for (auto nid = complex.vertices_begin(); nid != complex.vertices_end(); ++nid) {
            Vec2 p = complex.get_pos(*nid);
            
            if (temp_gdh::is_greater(p, ld) && temp_gdh::is_greater(ru, p)) {
                std::cout << nid->get_index() << " ";
                debug_num[0] = (int)nid->get_index();
            }
        }
        std::cout << std::endl;
    }
    
    if (LOG_FACE) {
        std::cout << "Face keys: ";
        for (auto fid = complex.faces_begin(); fid != complex.faces_end(); ++fid) {
            auto pts = complex.get_pos(*fid);
            bool inside = true;
            for (auto p : pts) {
                if (!(temp_gdh::is_greater(p, ld) && temp_gdh::is_greater(ru, p))) {
                    inside = false;
                }
            }
            if (inside) {
                std::cout << *fid << " ";
            }
        }
        std::cout << std::endl;
    }
}
