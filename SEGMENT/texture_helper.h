//
//  texture_helper.h
//  DSC
//
//  Created by Tuan Nguyen Trung on 2/11/15.
//  Copyright (c) 2015 Asger Nyman Christiansen. All rights reserved.
//

#ifndef __DSC__texture_helper__
#define __DSC__texture_helper__

#include <stdio.h>
#include "define.h"
#include "CImg.h"

#ifdef WIN32
#include <GL/glew.h>
#include <GL/glut.h>
#include <Util/ArgExtracter.h>
#else
#include <GEL/Util/ArgExtracter.h>
#include <GEL/GL/glew.h>
#include <GLUT/glut.h>
#endif


class texture_helper {
    
#define MAX_NB_TEXTURE 10
    
public:
    texture_helper();
    ~texture_helper();
    
    Vec2 get_tex_size(int tex_id){return tex_sizes[tex_id];};
    
    void map_texture(int tex_id);
    void end();
    
    void drawImage();
    
    unsigned int R(int x, int y);
    unsigned int G(int x, int y);
    unsigned int B(int x, int y);
    
private:
    std::vector<Vec2> tex_sizes;
    std::vector<GLuint> texture;
    
    // Load texture
    GLuint LoadTexture( const char * filename );
    
private:
    double * gray_image_;
    cimg_library::CImg<unsigned int> *image_;
    Vec2 _size;
};


#endif /* defined(__DSC__texture_helper__) */
