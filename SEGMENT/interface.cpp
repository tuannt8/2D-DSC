//
//  interface.cpp
//  DSC
//
//  Created by Tuan Nguyen Trung on 2/9/15.
//  Copyright (c) 2015 Asger Nyman Christiansen. All rights reserved.
//
#include "interface.h"

#include "trializer.h"
#include "object_generator.h"
#include "draw.h"

#include "dyn_integral.h"

#include "gl_debug_helper.h"
#include "dynamics_edge.h"

void _check_gl_error(const char *file, int line)
{
    GLenum err (glGetError());
    
    while(err!=GL_NO_ERROR) {
        std::string error;
        
        switch(err) {
            case GL_INVALID_OPERATION:      error="INVALID_OPERATION";      break;
            case GL_INVALID_ENUM:           error="INVALID_ENUM";           break;
            case GL_INVALID_VALUE:          error="INVALID_VALUE";          break;
            case GL_OUT_OF_MEMORY:          error="OUT_OF_MEMORY";          break;
            case GL_INVALID_FRAMEBUFFER_OPERATION:  error="INVALID_FRAMEBUFFER_OPERATION";  break;
        }
        
        std::cerr << "GL_" << error.c_str() <<" - "<<file<<":"<<line<<std::endl;
        err=glGetError();
    }
}

#define check_gl_error() _check_gl_error(__FILE__,__LINE__)


void display_(){
    interface::get_instance()->display();
}

void keyboard_(unsigned char key, int x, int y){
    interface::get_instance()->keyboard(key, x, y);
}

void reshape_(int width, int height){
    interface::get_instance()->reshape(width, height);
}

void visible_(int v){
    interface::get_instance()->visible(v);
}

void animate_(){
    interface::get_instance()->animate();
}

void glutMouseFunc_(int button, int state, int x, int y){
    gl_debug_helper::mouseDown(button, state, x, y);
};

void glutMotion_(int x, int y){
    gl_debug_helper::mouseMove(x, y);
};

interface* interface::instance = NULL;

#pragma mark - Glut display
void interface::display(){
    if (glutGet(GLUT_WINDOW_WIDTH) != WIN_SIZE_X || glutGet(GLUT_WINDOW_HEIGHT) != WIN_SIZE_Y) {
        return;
    }
    

    
    draw();
    update_title();
    
    check_gl_error();
    
    if (RUN) {
        dynamics_image_seg();
        glutPostRedisplay();
    }
}

void interface::animate(){
//    if (RUN) {
//        dynamics_image_seg();
//        glutPostRedisplay();
//    }
//    else{
//        sleep(0.7);
//    }
}

void interface::reshape(int width, int height){
    WIN_SIZE_X = width;
    WIN_SIZE_Y = height;
    if(dsc)
    {
        double image_ratio = imageSize[1] / imageSize[0];
        double gl_ratio = (double)WIN_SIZE_Y / WIN_SIZE_X;
        
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        gluOrtho2D(0, imageSize[0], 0, imageSize[1]);
        
        double lx = (gl_ratio < image_ratio)? WIN_SIZE_Y/image_ratio : WIN_SIZE_X;
        double ly = (gl_ratio < image_ratio)? WIN_SIZE_Y : WIN_SIZE_X*image_ratio;


        glViewport((WIN_SIZE_X - lx)/2, (WIN_SIZE_Y - ly)/2, lx, ly);
        glutReshapeWindow(WIN_SIZE_X, WIN_SIZE_Y);
        
        
        gl_debug_helper::coord_transform(Vec2((WIN_SIZE_X - lx)/2, (WIN_SIZE_Y - ly)/2),
                                         Vec2(imageSize[0] / lx, imageSize[1] / ly),
                                         WIN_SIZE_Y);
    }
}

void interface::visible(int v){
//    if(v==GLUT_VISIBLE){
//        glutIdleFunc(animate_);
//    }
//    else
//        glutIdleFunc(0);
}

void interface::keyboard(unsigned char key, int x, int y){
    
    switch (key) {
        case 'd':
            gl_debug_helper::change_state();
            break;
        case 'p':
            gl_debug_helper::print_debug_info_nearest(*dsc);
            gl_debug_helper::print_image_info(*image_);
            break;
        case 'i':
            std::cout << "Input debug number: ";
            std::cin >> debug_num[0];
            break;
        case ' ':
        {
            RUN = !RUN;
            
            //dynamics_image_seg();
        }
            break;
        case '\t':
            Painter::save_painting_no_overwite(WIN_SIZE_X, WIN_SIZE_Y, "./LOG");
            break;
        case 'u':
            std::cout << g_param.alpha << " - New alpha: ";
            std::cin >> g_param.alpha;
            std::cout << g_param.beta << " - New beta: ";
            std::cin >> g_param.beta;
            break;
        default:
            break;
    }
    
    int dis = (int)key - 48;
    if (dis <= 10 and dis >= 0) {
        bDiplay_[dis] = ! bDiplay_[dis];
    }
    g_param.bDisplay = bDiplay_;
    
    glutPostRedisplay();
}

void interface::initGL(){
    glutInitWindowSize(WIN_SIZE_X,WIN_SIZE_Y);
#ifdef WIN32
    glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_MULTISAMPLE);
#else
    glutInitDisplayString("rgba double samples=16");
#endif
    glutCreateWindow("");
    
    glEnable(GL_MULTISAMPLE);
    
    glEnable(GL_LINE_SMOOTH);
    glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
    glEnable(GL_POINT_SMOOTH);
    glHint(GL_POINT_SMOOTH_HINT, GL_NICEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    
    glutDisplayFunc(display_);
    glutKeyboardFunc(keyboard_);
    glutVisibilityFunc(visible_);
    glutReshapeFunc(reshape_);
    glutMouseFunc(glutMouseFunc_);
    glutMotionFunc(glutMotion_);
    
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}

#pragma mark - Interface
void interface::draw()
{
    Painter::begin();
    
    if (bDiplay_[1]) {
        image_->draw_image(WIN_SIZE_X);
    }
    
    if (bDiplay_[3] and dsc) {
        Painter::draw_faces(*dsc);
    }
    
    if (bDiplay_[2] and dsc) {
        glColor3f(1, 0.4, 0.3);
        Painter::draw_edges(*dsc);
        glColor3f(1, 0.0, 0.0);
        Painter::draw_vertices(*dsc);
    }
    

    
    if (!bDiplay_[4] and dsc) {
        Painter::draw_faces_intensity(*dsc);
    }
    
    
    if(bDiplay_[5]){
        Painter::draw_internal_force(*dsc);
    }
    
    if(bDiplay_[6]){
        Painter::draw_external_force(*dsc);
    }
    
    if(!bDiplay_[7]){
        glColor3f(1, 0, 0);
        // Painter::draw_vertices_index(*dsc);
        Painter::draw_faces_index(*dsc);
    }
    
    gl_debug_helper::draw();
    
    Painter::end();
}

std::vector<DSC2D::vec2> get_quad(double minx, double miny, double maxx, double maxy){
    std::vector<DSC2D::vec2> quad_tex;
    quad_tex.push_back(DSC2D::vec2(minx, maxy));
    quad_tex.push_back(DSC2D::vec2(maxx, maxy));
    quad_tex.push_back(DSC2D::vec2(maxx, miny));
    quad_tex.push_back(DSC2D::vec2(minx, miny));
    
    return quad_tex;
}

void interface::draw_coord(){
    DSC2D::DesignDomain const * domain = dsc->get_design_domain();
    std::vector<DSC2D::vec2> corners = domain->get_corners();
    DSC2D::vec2 min(INFINITY, INFINITY), max(-INFINITY, -INFINITY);
    for(auto p : corners){
        if (min[0] > p[0])
            min[0] = p[0];
        if (min[1] > p[1])
            min[1] = p[1];
        if (max[0] < p[0])
            max[0] = p[0];
        if (max[1] < p[1])
            max[1] = p[1];
    }
    
    DSC2D::vec2 center = (min + max)/2.0;
    DSC2D::vec2 length = (max - min);
    double l = std::min(length[0], length[1]);
    DSC2D::vec2 lx = center; lx[0] += l/2.0;
    DSC2D::vec2 ly = center; ly[1] += l/2.0;
    
    
    glLineWidth(2.0);
    glBegin(GL_LINES);
    
    glColor3f(1, 0, 0);
    glVertex2dv(center.get());
    glVertex2dv(lx.get());
    
    glColor3f(0, 1, 0);
    glVertex2dv(center.get());
    glVertex2dv(ly.get());
    
    glEnd();
    glLineWidth(1.0);
}

void interface::draw_image(){

    DSC2D::DesignDomain const * domain = dsc->get_design_domain();
    std::vector<DSC2D::vec2> corners = domain->get_corners();

    std::vector<DSC2D::vec2> quad_v = get_quad(0, 0, imageSize[0], imageSize[1]);
    std::vector<DSC2D::vec2> quad_tex;// = get_quad(0.0, 0.0, 1.0, 1.0);

    quad_tex.push_back(DSC2D::vec2(1, 0));
    quad_tex.push_back(DSC2D::vec2(1, 1));
    quad_tex.push_back(DSC2D::vec2(0, 1));
    quad_tex.push_back(DSC2D::vec2(0, 0));
    
    glColor3f(1, 1, 1);
    glBegin(GL_QUADS);
    for (int i = 0; i < 4; i++) {
        glVertex2dv((GLdouble*)quad_v[i].get());
        glTexCoord2dv((GLdouble*)quad_tex[i].get());
    }
    glEnd();
    
    glLineWidth(2.0);
    glColor3f(1, 0, 0);
    glBegin(GL_LINES);
    for (int i = 0; i < 4; i++) {
        glVertex2dv((GLdouble*)quad_v[i].get());
        glVertex2dv((GLdouble*)quad_v[(i+1)%4].get());
    }
    glEnd();
}

void interface::update_title()
{
    std::ostringstream oss;
    oss << "2D DSC\t";

    std::string str(oss.str());
    glutSetWindowTitle(str.c_str());
}

interface::interface(int &argc, char** argv){
    instance = this;
    WIN_SIZE_X = 900;
    WIN_SIZE_Y = 600;
    
    bDiplay_.resize(10);
    std::fill(bDiplay_.begin(), bDiplay_.end(), true);
    
    glutInit(&argc, argv);
    initGL();
    
    dyn_ = std::unique_ptr<dynamics_mul>(new dynamics_mul);
    dsc = nullptr;
    
    image_ = std::unique_ptr<image>(new image);
    if (argc > 1) {
        image_->load_image(std::string(argv[1]));
    }else
    image_->load_image(std::string(DATA_PATH) + IMAGE_NAME);
    imageSize = Vec2(image_->width(), image_->height());

    check_gl_error();
    
    init_dsc();
    
    gl_debug_helper::set_dsc(&(*dsc));
//    init_sqaure_boundary();
//   init_boundary();
    
    reshape(WIN_SIZE_X, WIN_SIZE_Y);
    display();
}

#pragma mark - Data
using namespace DSC2D;

void interface::init_dsc(){
    double width = imageSize[0];
    double height = imageSize[1];
    
    DISCRETIZATION = (double) height / (double)DISCRETIZE_RES;
    
    width -= 2*DISCRETIZATION;
    height -= 2*DISCRETIZATION;
    
//    int width = 600;
//    int height = 400;
//    DISCRETIZATION = 100;
    
    
    
    std::vector<real> points;
    std::vector<int> faces;
    Trializer::trialize(width, height, DISCRETIZATION, points, faces);
    
    DesignDomain *domain = new DesignDomain(DesignDomain::RECTANGLE, width, height, 0 /* DISCRETIZATION */);
    
    dsc = std::unique_ptr<DeformableSimplicialComplex>(
                            new DeformableSimplicialComplex(DISCRETIZATION, points, faces, domain));
    
 //   thres_hold_init();
    
    printf("Average edge length: %f \n", dsc->get_avg_edge_length());
}

void interface::thres_hold_init(){
    for (auto fid = dsc->faces_begin(); fid != dsc->faces_end(); fid++) {
        auto tris = dsc->get_pos(*fid);
        int totalPixel = 0;
        double total_inten = 0.0;
        image_->get_tri_intensity(tris, & totalPixel, & total_inten);
        double mean_c = total_inten / (double)totalPixel;
        
        if (mean_c < 0.2) {
            dsc->set_label(*fid, 0);
        }
        else if (mean_c < 0.7)
            dsc->set_label(*fid, 1);
        else
            dsc->set_label(*fid, 3);
    }
}

void interface::init_sqaure_boundary(){
    Vec2 s = imageSize;// tex->get_image_size();
    double left = 0.2;
    double right = 0.6;
    ObjectGenerator::create_square(*dsc, vec2(left*s[0], left*s[1]), vec2(right*s[0], right*s[1]), 1);
}

void interface::init_boundary(){
    std::vector<DSC2D::DeformableSimplicialComplex::face_key> faceKeys;
    for (auto p = dsc->faces_begin(); p != dsc->faces_end(); p++) {
        // Compute average intensity inside triangle
        auto pts = dsc->get_pos(*p);
        int count;
//        if(image_->get_triangle_intensity_count(pts, &count) > 0.1*count*MAX_BYTE ){
//            faceKeys.push_back(*p);
//        }
    }
    
    ObjectGenerator::label_tris(*dsc, faceKeys, 1);
}

void interface:: dynamics_image_seg(){
//    dyn_->update_dsc(*dsc, *image_);
    
    static dyn_integral dyn;
    dyn.update_dsc(*dsc, *image_);

//    static dynamics_edge dyn;
//    dyn.update_dsc(*dsc, *image_);
}