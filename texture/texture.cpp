//
//  texture.cpp
//  DSC_texture_seg
//
//  Created by Tuan Nguyen Trung on 08/03/16.
//  Copyright © 2016 Asger Nyman Christiansen. All rights reserved.
//

#include <stdio.h>
#include "texture.h"
#include <algorithm>
#include <math.h>
#include <stdio.h>
#include <vector>
#include "define.h"

#include "profile.h"
#include "sparse_mat.h"
#include "profile.h"

#include "setting_file.h"

#define EPS std::numeric_limits<double>::epsilon()

namespace texture
{

    
    using namespace std;
    using namespace cimg_library;
    using namespace arma;
    
    Eigen::VectorXd  dictionary::compute_probability(const Eigen::VectorXd & labeled)
    {
        profile t("Update dictionary");
            
        auto ll1 = _T1_s*labeled;
        
        auto ll2 = _T2_s*ll1;

        return ll2;
    }
    
    Eigen::VectorXd dictionary::compute_probability_T1(const Eigen::VectorXd & labeled)
    {
        return _T1_s*labeled;
    }
    

    void dictionary::preprocess_mat()
    {
        {
        std::vector<double> row_count_1(B_height, 0);
        for (int k=0; k<_T1_s.outerSize(); ++k)
            for (Eigen::SparseMatrix<double>::InnerIterator it(_T1_s,k); it; ++it)
            {
                row_count_1[it.row()] += 1;
            }
        for (int k=0; k<_T1_s.outerSize(); ++k)
            for (Eigen::SparseMatrix<double>::InnerIterator it(_T1_s,k); it; ++it)
            {
                it.valueRef() = it.value() / (row_count_1[it.row()] + EPS);
            }
        }
        
        {
        std::vector<double> row_count_2(B_width, 0);
        for (int k=0; k<_T2_s.outerSize(); ++k)
            for (Eigen::SparseMatrix<double>::InnerIterator it(_T2_s,k); it; ++it)
            {
                row_count_2[it.row()] += 1;
            }
        for (int k=0; k<_T2_s.outerSize(); ++k)
            for (Eigen::SparseMatrix<double>::InnerIterator it(_T2_s,k); it; ++it)
            {
                it.valueRef() = it.value() / (row_count_2[it.row()]+ EPS);
            }
        }
    }
    
    void dictionary::log_mat_col_major(double *m, int nrow, int ncol, char * mes)
    {
        for(int r = 0; r < nrow; r++)
        {
            for(int c = 0; c < ncol; c++)
            {
                cout << m[c*nrow + r] << " ";
            }
            cout << endl;
        }
    }
    
    arma::vec dictionary::multiply(const std::vector<ij> &  Bij, std::vector<double> & row_val, const arma::vec & x, long num_row)
    {
        arma::vec out = zeros(num_row);
        
        auto e = Bij.begin();
        while (e != Bij.end())
        {
            auto r = e->i;
            
            while (1)
            {
                auto c = e->j;
                out[r] += row_val[r]*x[c];
                
                e++;
                if (e == Bij.end() or e->i != r)
                {
                    break;
                }
            }
        }
        
        return out;
    }
    
    dictionary::dictionary(std::string imName)
    {
        profile * t = new profile("Build dictionary");
        
        std::cout << "Building dictionary" << std::endl;
        // Load test image
        imaged im;
        im.load(imName.data());

        
        int width = im.width();
        int height = im.height();
        

        // Parametters
        double Md = setting_file.batch_size;             // batch size
        double bd = setting_file.branching_factor;              // branching factor
        double n_train_d = setting_file.num_training_patch;    // training patch
        double Ld = setting_file.num_layer;              // number of layer
        bool normalize = setting_file.normalize;
        
        int ndim;
        int dim[3] = {height, width, 3};
        if(setting_file._b_color)
        {
            ndim = 3;
        }
        else{
            ndim = 1;
        }
        
        double * A;
        
        {
//            double * I = im.data();
            // Convert from column major to row major
            double * I = new double[width*height*ndim];
            for (int i = 0; i < ndim; i++)
            {
                int off = i*width*height;
                for (int r = 0; r < height; r++)
                {
                for (int c = 0; c < width; c++)
                {

                        I[off + c*height + r] = im(c, r, 0, i);
                    }
                }
            }
            
            std::cout << "Building tree" << std::endl;
            // Build tree
            
            
            int treeDim[2];
            double * tree =  build_tree( &I[0], &Md, &bd, &n_train_d, &Ld, ndim, dim, treeDim, normalize);


            
            std::cout << "Search tree" << std::endl;
            // Build matrix A; same dimension with image
            A = search_tree(&I[0], tree, &bd, ndim, 2, dim, treeDim, normalize);
            
//            log_mat_col_major(A, height, width, "----A");
            
            delete tree; // release memory
        }
        
        std::cout << "Building adjacency matrix" << std::endl;
        // Matrix B
        auto Bij2 = biadjacency_matrix(A, height, width, Md);
        
        // Using armadillo to compute the matrix
        B_width = width*height;
        int K = 0;
        for (int a=0; a<width*height; a++)
            if (A[a]>K)
                K = A[a];
        B_height = Md*Md*K;
        

        
        delete A; // release memory
        
        delete t;
        
        t = new profile("Compute T1, T2");
        
        std::cout << "Build adjacency 1" << std::endl;
        
        _T2_s = Eigen::SparseMatrix<double>(B_width, B_height);
        _T2_s.setFromTriplets(Bij2.begin(), Bij2.end());
        
        _T1_s = _T2_s.transpose();
 
        std::cout << "Post process" << std::endl;
        preprocess_mat();

        
        delete t;
    }
}

















































