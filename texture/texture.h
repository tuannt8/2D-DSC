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
#include "define.h"
#include <armadillo>
#include <string>
#include "sparse_mat.h"
#include "smooth_image.hpp"



#ifdef Success
#undef Success
#endif

#include "Eigen/SparseCore"

//#define TUAN_TEST

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
    
//    struct ij
//    {
//        int i,j;
//        ij(int i_, int j_) : i(i_), j(j_){}; // constructor
//        bool operator<(const ij second) const{ // leq operator needed for sorting
//            return (j<second.j) || ((j==second.j) && (i<second.i));
//        }    
//    };


    double * search_tree(
                     double *I // input, image
                     , double * tree // input, tree
                     , double *bd // input, branching_factor
                     , int ndim // input, number of image dimensions
                     , int ndtree // input, number of tree dimension
                     , int * dim // image dimension
                     , int * dtree // tree dimension
    , bool normalize 
    );

    double * build_tree(double *I, // image, input
                    double *Md, // input, patch size
                    double *bd, // input, branching_factor
                    double *n_train_d, //input, number_training_patches
                    double * Ld, // input, number_layers
                    
                    int ndim, // input, image layer
                    int *dim // input, image dimension
    , int treeDim[]
                        , bool normalize
    );

    std::vector<Eigen::Triplet<double>> biadjacency_matrix(
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

        Eigen::VectorXd compute_probability(const Eigen::VectorXd & labeled);//{return _T2*(_T1*labeled);};
        Eigen::VectorXd compute_probability_T1(const Eigen::VectorXd & labeled);
        Eigen::VectorXd compute_probability_T2(const Eigen::VectorXd & labeled){return _T2_s*labeled;}
        
    private:
        // The dictionary matrix
        std::vector<double> T1_row_count, T2_row_count;
        
        std::map<std::pair<long,long>, double> T21;
        
//        std::vector<ij> Bij1, Bij2;
        long B_width, B_height;
        
        arma::vec multiply(const std::vector<ij> &  Bij, std::vector<double> & row_val, const arma::vec & x, long num_row);
        
        void preprocess_mat();
        
        
        Eigen::SparseMatrix<double> _T1_s, _T2_s;
    public:
        void log_mat_col_major(double *m, int nrow, int ncol, char * mes = "----");
        smooth_image mapping_img;
    };
}

#endif /* texture_h */

