//
//  texture.h
//  DSC_texture_seg
//
//  Created by Tuan Nguyen Trung on 08/03/16.
//  Copyright © 2016 Asger Nyman Christiansen. All rights reserved.
//

#ifndef texture_h
#define texture_h
#include <vector>
#include <stdlib.h>
#include "CImg.h"
#include <armadillo>
#include <string>

namespace texture
{
    typedef unsigned char BYTE;
    #define MAX_BYTE 255
    
    typedef cimg_library::CImg<BYTE> imageb;
    typedef cimg_library::CImg<double> imaged;

    // struct for the tree
    struct tree_st
    {
        double *tree_data;
        int n_dim, n_nodes, branch_fac, M, Mh;
    };

    // struct for image
    struct im_st
    {
        double *im_data;
        int rows, cols, layers, n_pix;
    };
    
    struct ij
    {
        int i,j;
        ij(int i_, int j_) : i(i_), j(j_){}; // constructor
        bool operator<(const ij second) const{ // leq operator needed for sorting
            return (j<second.j) || ((j==second.j) && (i<second.i));
        }    
    };


    double * search_tree(
                     double *I // input, image
                     , double * tree // input, tree
                     , double *bd // input, branching_factor
                     , int ndim // input, number of image dimensions
                     , int ndtree // input, number of tree dimension
                     , int * dim // image dimension
                     , int * dtree // tree dimension
    );

    double * build_tree(double *I, // image, input
                    double *Md, // input, patch size
                    double *bd, // input, branching_factor
                    double *n_train_d, //input, number_training_patches
                    double * Ld, // input, number_layers
                    
                    int ndim, // input, image layer
                    int *dim // input, image dimension
    , int treeDim[]
    );

    std::vector<ij> biadjacency_matrix(
                            double *A // input, assignment image
                            , int X // input, image size X
                            , int Y // input, image size Y
                            , int M // input, patch size

    );

    void test();
 
    void log_im(double *I, int width, int height, char* name);
    
    /*
     Class dictionary
     Build base on image range [0:1]
     */
    class dictionary{
    public:
        dictionary(std::string imName);
        ~dictionary(){}
        
        void save_matrix(std::string imName);
        bool load_matrix(std::string imName);
        
        std::string get_back_up_name(const std::string file);
        void back_up(std::string imName, bool overwrite = false);
        bool load_up(std::string imName);
    private:
        // The dictionary matrix
        arma::Mat<double> _T1, _T2;
    public:
    };
}

#endif /* texture_h */

