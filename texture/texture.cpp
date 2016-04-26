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

namespace texture
{
    using namespace std;
    using namespace cimg_library;
    using namespace arma;
    
    arma::vec  dictionary::compute_probability(const arma::vec & labeled)
    {
        arma::vec out;
        // T1*labeled
        out = multiply(Bij1, T1_row_count, labeled, B_height);
        
        // T2*
        out = multiply(Bij2, T2_row_count, out, B_width);
        
        return out;
    }
    
    void dictionary::preprocess_mat()
    {
        // T1;
        T1_row_count = vector<double>(B_height, 0);
        
        for (auto & ij : Bij1)
        {
            T1_row_count[ij.i]+=1;
        }
        
        for (auto & tt1 : T1_row_count)
        {
            tt1 = 1.0/(tt1 + 0.00001);
        }
        
        // T2
        T2_row_count = vector<double>(B_width, 0);
        for (auto & ij : Bij2)
        {
            T2_row_count[ij.i]+=1;
        }
        
        for (auto & tt2 : T2_row_count)
        {
            tt2 = 1.0/(tt2 + 0.00001);
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
        std::cout << "Building dictionary" << std::endl;
        // Load test image
        imaged im;
        im.load(imName.data());
        im.normalize(0, 1);
        
        int width = im.width();
        int height = im.height();
        

        // Parametters
        double Md = 15;             // batch size
        double bd = 5;              // branching factor
        double n_train_d = 5000;    // training patch
        double Ld = 4;              // number of layer
        
        // Image information
        //        int ndim = im.spectrum();
        //        int dim[3] = {width, height, 3};
        int ndim = 1;
        int dim[3] = {width, height};
        
        double * A;
        
        {
            double * I = im.data();
            
            std::cout << "Building tree" << std::endl;
            // Build tree
            
            
            int treeDim[2];
            double * tree =  build_tree( &I[0], &Md, &bd, &n_train_d, &Ld, ndim, dim, treeDim);
            
            std::cout << "Search tree" << std::endl;
            // Build matrix A; same dimension with image
            A = search_tree(&I[0], tree, &bd, ndim, 2, dim, treeDim);
            
            delete tree; // release memory
        }
        
        std::cout << "Building adjacency matrix" << std::endl;
        // Matrix B
        Bij2 = biadjacency_matrix(A, width, height, Md);
        
        // Using armadillo to compute the matrix
        B_width = width*height;
        int K = 0;
        for (int a=0; a<width*height; a++)
            if (A[a]>K)
                K = A[a];
        B_height = Md*Md*K;
        
        delete A; // release memory
        
        std::cout << "Build adjacency 1" << std::endl;
        
        Bij1.reserve(Bij2.size());
        for (auto & _ij : Bij2)
        {
            Bij1.push_back(ij(_ij.j, _ij.i));
        }
        std::sort(Bij1.begin(), Bij1.end());
        
        std::cout << "Preprocess" << std::endl;
        preprocess_mat();
    }
    

    
}

















































