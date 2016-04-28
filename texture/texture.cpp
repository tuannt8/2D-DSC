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

namespace texture
{
    using namespace std;
    using namespace cimg_library;
    using namespace arma;
    
    Eigen::VectorXd  dictionary::compute_probability(const Eigen::VectorXd & labeled)
    {
//        arma::vec out;
//        {
//            profile t("std vector multiply");
//        // T1*labeled
//        out = multiply(Bij1, T1_row_count, labeled, B_height);
//        
//        // T2*
//        out = multiply(Bij2, T2_row_count, out, B_width);
//        }
        
        // With Eigen
//            profile t("eigen multiply");
//            Eigen::SparseVector<double> label_e(labeled.n_rows);
//            for (int i  = 0; i < labeled.n_rows; i++)
//            {
//                if (labeled[i] != 0)
//                {
//                    label_e.insert(i) = labeled[i];
//                }
//            }
            
        auto ll1 = _T1_s*labeled;
        auto ll2 = _T2_s*ll1;

        
        return ll2;
    }
    
    void dictionary::preprocess_mat()
    {
//        // T1;
//        T1_row_count = vector<double>(B_height, 0);
//        
//        for (auto & ij : Bij1)
//        {
//            T1_row_count[ij.i]+=1;
//        }
//        
//        for (auto & tt1 : T1_row_count)
//        {
//            tt1 = 1.0/(tt1 + 0.00001);
//        }
//        
//        // T2
//        T2_row_count = vector<double>(B_width, 0);
//        for (auto & ij : Bij2)
//        {
//            T2_row_count[ij.i]+=1;
//        }
//        
//        for (auto & tt2 : T2_row_count)
//        {
//            tt2 = 1.0/(tt2 + 0.00001);
//        }
        
        std::vector<double> row_count_1(B_height, 0);
        for (int k=0; k<_T1_s.outerSize(); ++k)
            for (Eigen::SparseMatrix<double>::InnerIterator it(_T1_s,k); it; ++it)
            {
                row_count_1[it.row()] += 1;
            }
        for (int k=0; k<_T1_s.outerSize(); ++k)
            for (Eigen::SparseMatrix<double>::InnerIterator it(_T1_s,k); it; ++it)
            {
                it.valueRef() = it.value() / (row_count_1[it.row()] + 0.00001);
            }
        
        std::vector<double> row_count_2(B_height, 0);
        for (int k=0; k<_T2_s.outerSize(); ++k)
            for (Eigen::SparseMatrix<double>::InnerIterator it(_T2_s,k); it; ++it)
            {
                row_count_2[it.row()] += 1;
            }
        for (int k=0; k<_T2_s.outerSize(); ++k)
            for (Eigen::SparseMatrix<double>::InnerIterator it(_T2_s,k); it; ++it)
            {
                it.valueRef() = it.value() / (row_count_1[it.row()]+ 0.00001);
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
        auto Bij2 = biadjacency_matrix(A, width, height, Md);
        
        // Using armadillo to compute the matrix
        B_width = width*height;
        int K = 0;
        for (int a=0; a<width*height; a++)
            if (A[a]>K)
                K = A[a];
        B_height = Md*Md*K;
        
        delete A; // release memory
        
        std::cout << "Build adjacency 1" << std::endl;
        
        _T2_s = Eigen::SparseMatrix<double>(B_width, B_height);
        _T2_s.setFromTriplets(Bij2.begin(), Bij2.end());
        
        _T1_s = _T2_s.transpose();
        
        std::cout << "Post process" << std::endl;
        preprocess_mat();
        
//        Bij1.reserve(Bij2.size());
//        for (auto & _ij : Bij2)
//        {
//            Bij1.push_back(ij(_ij.j, _ij.i));
//        }
//        std::sort(Bij1.begin(), Bij1.end());
//        
//        std::cout << "Preprocess" << std::endl;
//        preprocess_mat();
//        
//        // Test with Eigen
//        std::vector<Eigen::Triplet<double>> bb1, bb2;
//        
//        for (auto ije : Bij1)
//        {
//            bb1.push_back(Eigen::Triplet<double>(ije.i, ije.j, 1));
//        }
//        for (auto ije : Bij2)
//        {
//            bb2.push_back(Eigen::Triplet<double>(ije.i, ije.j, 1));
//        }
//        
//        _T1_s = Eigen::SparseMatrix<double>(B_height,B_width);
//        _T1_s.setFromTriplets(bb1.begin(), bb1.end());
//        _T2_s = Eigen::SparseMatrix<double>(B_width, B_height);
//        _T2_s.setFromTriplets(bb2.begin(), bb2.end());

    }
    

    
}

















































