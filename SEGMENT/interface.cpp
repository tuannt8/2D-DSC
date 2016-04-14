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
#include "adapt_mesh.h"

#include "options_disp.h"
#include "../texture/texture.h"
#include "setting_file.h"

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
    options_disp::mouse_func(button, state, x, y);
};;


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
    
    double real_width = width - options_disp::width_view;
    if(dsc)
    {
        double image_ratio = imageSize[1] / imageSize[0];
        double gl_ratio = (double)WIN_SIZE_Y / real_width;
        
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        gluOrtho2D(0, imageSize[0], 0, imageSize[1]);
        
        double lx = (gl_ratio < image_ratio)? WIN_SIZE_Y/image_ratio : real_width;
        double ly = (gl_ratio < image_ratio)? WIN_SIZE_Y : real_width*image_ratio;


        glViewport(options_disp::width_view + (real_width - lx)/2, (WIN_SIZE_Y - ly)/2, lx, ly);
      //  glutReshapeWindow(WIN_SIZE_X, WIN_SIZE_Y);
        
        
        gl_debug_helper::coord_transform(Vec2(options_disp::width_view + (real_width - lx)/2, (WIN_SIZE_Y - ly)/2),
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
            break;
        case 'r':
        {
            _tex_seg->update_probability();
        }
            break;
        case ' ':
        {
            RUN = !RUN;
            update_title();
            //dynamics_image_seg();
            break;
        }

        case '\t':
            Painter::save_painting_no_overwite(WIN_SIZE_X, WIN_SIZE_Y, "./LOG");
            break;
        case 'i':
            dsc->increase_resolution_range();
            break;
        case 'f': // Flipping phase
        {

        }
            break;
        case 's': // Split edge
        {

        }
            break;
        case 'b': // Split edge
        {
            back_up_dsc();
        }
            break;
        case 'l': // Split edge
        {
            load_dsc();
        }
            break;
        case 'w': // Split edge
        {
//            dyn_->write_energy();
        }
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

using namespace DSC2D;
void interface::load_dsc()
{
    std::ostringstream os;
    os << LOG_PATH << "dsc.dsc";
    
    std::ifstream myfile(os.str().c_str());
    
    if (myfile.is_open()) {
        int nb_vertice, nb_face;
        myfile >> nb_vertice;
        myfile >> nb_face;
        
        std::vector<double> points(3*nb_vertice, 0);
        for (int i = 0; i < nb_vertice; i++) {
            myfile >> points[3*i];
            myfile >> points[3*i+1];
        }
        std::vector<int> faces; faces.resize(3*nb_face);
        for (int i = 0; i < nb_face; i++) {
            myfile >> faces[3*i];
            myfile >> faces[3*i + 1];
            myfile >> faces[3*i + 2];
        }
        
        // Init DSC
        double width = imageSize[0];
        double height = imageSize[1];
        
        DISCRETIZATION = (double) height / (double)DISCRETIZE_RES;
        
        width -= 2*DISCRETIZATION;
        height -= 2*DISCRETIZATION;
        
        DesignDomain *domain = new DesignDomain(DesignDomain::RECTANGLE, width, height, 0 /* DISCRETIZATION */);
        
        dsc = std::shared_ptr<DeformableSimplicialComplex>(
                                                           new DeformableSimplicialComplex(DISCRETIZATION, points, faces, domain));

        gl_debug_helper::set_dsc(&(*dsc));
        
        myfile.close();
    }else{
        std::cout << "Fail to load dsc mesh \n";
    }
}
void interface::back_up_dsc()
{
    std::ostringstream os;
    os << LOG_PATH << "dsc.dsc";
    
    dsc->save(os.str().c_str());
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
    
    reshape(WIN_SIZE_X, WIN_SIZE_Y);
    
    
    
    if (options_disp::get_option("Image", true) and _origin_img) {
        _origin_img->draw_image();
    }
    
    if (options_disp::get_option("Test corrdinate", false) and _tex_seg) {
        _tex_seg->draw_test_coord();
    }
    
    if (options_disp::get_option("Labeled", false) and _origin_img) {
        _tex_seg->draw_dictionary();
    }
    
    if (options_disp::get_option("Probability", false) and _origin_img) {
        _tex_seg->draw_probability();
    }
    
    if (options_disp::get_option("DSC faces", true) and dsc) {
        Painter::draw_faces(*dsc);
    }
    
    if (options_disp::get_option("Face intensity", false) and dsc) {
        Painter::draw_faces_intensity(*dsc);
    }

    if (options_disp::get_option("Edge and vertices ", true) and dsc) {
        glLineWidth(1.0);
        Painter::draw_edges(*dsc);
        glColor3f(1, 0.0, 0.0);
        glPointSize(1.0);
    //    Painter::draw_vertices(*dsc);
    }

    if(options_disp::get_option("Phase index", false)){
        Painter::draw_face_label(*dsc);
    }

    if (options_disp::get_option("External force", false))
    {
        Painter::draw_external_force(*dsc);
    }
    
    if(options_disp::get_option("Vertices index", false)){
        glColor3f(1, 0, 0);
        Painter::draw_vertices_index(*dsc);
    }
    
    if(options_disp::get_option("Edge index", false)){
        glColor3f(1, 0, 0);
        // Painter::draw_vertices_index(*dsc);
        // Painter::draw_faces_index(*dsc);
        Painter::draw_edges_index(*dsc);
    }
    
    if (options_disp::get_option("Face index")){
        Painter::draw_faces_index(*dsc);
    }

    
    gl_debug_helper::draw();
    
    options_disp::draw(WIN_SIZE_X, WIN_SIZE_Y);
    
    Painter::end();
}

void interface::draw_tri_variant(){
}

void interface::draw_edge_energy(){
}


void interface::draw_test(){
    

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
}

void interface::update_title()
{
    std::ostringstream oss;
    oss << "2D DSC\t";
    if (RUN) {
        oss << " running\t";
    }

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
    
    _tex_seg = std::shared_ptr<texture_segment>(new texture_segment);
    _tex_seg->init();
    
    _origin_img = _tex_seg->_origin_img;
    imageSize = Vec2(_origin_img->width(), _origin_img->height());

    init_dsc();
    dsc->deform();
    
    _tex_seg->_dsc = dsc;
    _tex_seg->init_dsc_phases();
    
    gl_debug_helper::set_dsc(&(*dsc));
    
    reshape(WIN_SIZE_X, WIN_SIZE_Y);
    display();
}

#pragma mark - Data
using namespace DSC2D;

void interface::init_dsc(){
    double width = imageSize[0];
    double height = imageSize[1];
    
    DISCRETIZATION = (double) height / (double)setting_file.dsc_discretization;
    
    width -= 2*DISCRETIZATION;
    height -= 2*DISCRETIZATION;
    
    std::vector<real> points;
    std::vector<int> faces;
    Trializer::trialize(width, height, DISCRETIZATION, points, faces);
    
    width += 2*DISCRETIZATION;
    height += 2*DISCRETIZATION;
    DesignDomain *domain = new DesignDomain(DesignDomain::RECTANGLE, width, height, 0 /*,  DISCRETIZATION */);
    
    dsc = std::shared_ptr<DeformableSimplicialComplex>(
                            new DeformableSimplicialComplex(DISCRETIZATION, points, faces, domain));
    
    dsc->set_smallest_feature_size(SMALLEST_SIZE);
}

void interface::thres_hold_init(){

}

void interface::init_sqaure_boundary(){

}

void interface::init_boundary(){
//    std::vector<DSC2D::DeformableSimplicialComplex::face_key> faceKeys;
//    for (auto p = dsc->faces_begin(); p != dsc->faces_end(); p++) {
//        // Compute average intensity inside triangle
//        auto pts = dsc->get_pos(*p);
//        int count;
////        if(image_->get_triangle_intensity_count(pts, &count) > 0.1*count*MAX_BYTE ){
////            faceKeys.push_back(*p);
////        }
//    }
//    
//    ObjectGenerator::label_tris(*dsc, faceKeys, 1);
}

void interface::dynamics_image_seg(){

//    dyn_->update_dsc(*dsc, *image_);
    _tex_seg->update_dsc();

}


void interface::init_boundary_brain(){

}
