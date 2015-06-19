//
//  console_debug.h
//  DSC
//
//  Created by Tuan Nguyen Trung on 6/18/15.
//  Copyright (c) 2015 Asger Nyman Christiansen. All rights reserved.
//

#ifndef DSC_console_debug_h
#define DSC_console_debug_h
#include <thread>
#include <map>
#include <stdlib.h>
#include <string>

#ifdef WIN32
#include <GL/glut.h>
#else
#include <GLUT/glut.h>
#endif


class console_debug{
    class vec2{
    public:
        vec2(){};
        vec2(double c1, double c2){x_[0] = c1; x_[1] = c2;}
        ~vec2(){}
        double x_[2];
        
        double & x(){return x_[0];}
        double & y(){return x_[1];}
        double & operator[](int idx){return x_[idx];}
        vec2 operator+(vec2 r){
            vec2 o;
            o.x_[0] = x_[0] + r.x_[0];
            o.x_[1] = x_[1] + r.x_[1];
            return o;
        }
        vec2 & operator=(vec2 a){
            x_[0] = a.x_[0];
            x_[1] = a.x_[1];
            return *this;
        }
    };
    
    
    std::map<std::string, bool> options;

    const double WIN_SIZE_X = 600;
    const double WIN_SIZE_Y = 500;
    const double box_x = 200;
    const double box_y = 20;
    
    int cur_over;
    const double mouse_over[3] = {1,0,1};
    const double check[3] = {0,0,1};
    const double un_check[3] = {1,0,0};
    
private:
    // Call back
    void (*update[10])(void) = {nullptr};
    void update_display();
public:
    static void set_call_back(void (*func)(void)){
        static int idx = 0;
        console_debug::get_instance()->update[idx++] = (func);
    }
    
private:
    void RenderString(vec2 p, const std::string &string);
    void draw_box(int idx, std::string name, bool state);
public:
    static console_debug *instance;
    
    static console_debug * get_instance(){
        if (!instance) {
            instance = new console_debug;
        }
        return instance;
    }
    static bool get_opt (std::string op, bool defalt){
        return get_instance()->getOpt(op, defalt);
    }
    
private:
    console_debug(){
        
    }
public:
    
    ~console_debug(){
    };
    
    bool getOpt (std::string op, bool defalt){
        if (options.find(op) == options.end()) {
            options.insert(std::make_pair(op, defalt));
        }
        
        return options[op];
    }
    
    void init();
    
    void display();
    
    void animate();
    
    void reshape(int width, int height);
    
    void visible(int v);
    
    void keyboard(unsigned char key, int x, int y);
    
    void mouse_func(int button, int state, int x, int y);
};


#endif
