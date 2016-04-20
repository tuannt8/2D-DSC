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

namespace texture
{
    using namespace std;
    using namespace cimg_library;
    using namespace arma;
    
    arma::vec  dictionary::compute_probability(const arma::vec & labeled)
    {
        arma::vec out;
        // T1*labeled
        vector<double> T1_row_count(B_height, 0);
        
        
        for (auto & ij : Bij1)
        {
            T1_row_count[ij.i]+=1;
        }
        
        for (auto & tt1 : T1_row_count)
        {
            tt1 = 1.0/(tt1 + 0.00001);
        }
        
        
        out = multiply(Bij1, T1_row_count, labeled, B_height);
        
        // T2*
        vector<double> T2_row_count(B_width, 0);
        for (auto & ij : Bij2)
        {
            T2_row_count[ij.i]+=1;
        }
        
        for (auto & tt2 : T2_row_count)
        {
            tt2 = 1.0/(tt2 + 0.00001);
        }
        
        out = multiply(Bij2, T2_row_count, out, B_width);
        
        return out;
    }
    
    arma::vec dictionary::multiply(const std::vector<ij> &  Bij, std::vector<double> row_val, const arma::vec & x, long num_row)
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
                if (e == Bij.end() or
                    e->i != r)
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
        
//        double Md = 3;             // batch size
//        double bd = 5;              // branching factor
//        double n_train_d = 100;    // training patch
//        double Ld = 4;              // number of layer
        
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
        vector<ij> Bij = biadjacency_matrix(A, width, height, Md);
        
        // Using armadillo to compute the matrix
        B_width = width*height;
        int K = 0;
        for (int a=0; a<width*height; a++)
            if (A[a]>K)
                K = A[a];
        B_height = Md*Md*K;
        
        delete A; // release memory
        
        Bij2 = Bij;
        
        Bij1.resize(Bij.size());
        for (int i = 0; i < Bij.size(); i++)
        {
            auto & _ij = Bij[i];
            Bij1[i] = ij(_ij.j, _ij.i);
        }
        std::sort(Bij1.begin(), Bij1.end());
        
//        std::cout << "Building T2*T1" << std::endl;
//        
//        auto ij = Bij.begin();
//        while (ij != Bij.end())
//        {
//            auto c = ij->j;
//            
//            vector<long> local_colum;
//            
//            while (1)
//            {
//                auto r = ij->i;
//                local_colum.push_back(r);
//                
//                ij ++;
//                if (ij->j != c)
//                {
//                    break;
//                }
//            }
//            
//            // multiply
//            for (auto lc1 : local_colum)
//            {
//                for (auto lc2 : local_colum)
//                {
//                    // T21(lc1,lc2) = T2(lc1,c)*T1(c,lc2);
//                    double v = T2_row_count[lc1]*T1_row_count[lc2];
//                    
//                    auto a = T21.find(std::make_pair(lc1, lc2));
//                    if (a == T21.end())
//                    {
//                        T21.insert(std::make_pair(std::make_pair(lc1, lc2), v));
//                    }
//                    else
//                    {
//                        a->second += v;
//                    }
//                }
//            }
//        }
        
//        Mat<double> B(B_width, B_height);
     //   Mat<double> Bt(B_height, B_width);
//        
//        std::cout << "Compute T1 and T2" << std::endl;
//        
//        auto _T1 = SpMat<double>(B_height, B_width); // Bt
//        auto _T2 = SpMat<double>(B_width, B_height); // B
//        
//        vec B1 = zeros(B_width);
//        vec Bt1 = zeros(B_width);
//        
//        for (auto & ij : Bij)
//        {
//            _T2(ij.i, ij.j) = 1;
////            B1(ij.i) ++;
////            
////            _T1(ij.j, ij.i) = 1;
////            Bt1(ij.j)++;
//        }
//
//        
//        std::cout << "Normalize T1 and T2" << std::endl;
////        _T1 = zeros(Bt.n_rows, Bt.n_cols);
//        
//        for(auto ij : Bij)
//        {
//            _T1(ij.j, ij.i) = _T1(ij.j, ij.i)/(Bt1(ij.j) + 0.00001);
//        }
//        
////        Bt = Mat<double>(); // release memory
////        Bt1 = vec(); // release memory
//        
//        
////        _T2 = zeros(B.n_rows, B.n_cols);
//        for(auto ij : Bij)
//        {
//            _T2(ij.i, ij.j) = _T2(ij.i, ij.j)/(B1(ij.i) + 0.00001);
//        }
//        
//        //        back_up(imName);
//        std::cout << "Dictionary built" << std::endl;
    }
    
    
    void test()
    {

        profile t1("Begin to bij");
    
//        printf("Test build probability \n");
        
        // Load test image
        imageb im;
        im.load("./DATA/leopard.png");
        
        int width = im.width();
        auto height = im.height();
//        printf("Loaded image, %d x %d \n", im.width(), im.height());
        
        // Image 1 layer
        vector<double> I;
        I.resize(width*height);
        for (int i = 0; i < height; i++)
        {
            for(int j = 0; j < width; j++)
            {
                I[i*width + j] = (double)im(j,i) / MAX_BYTE;
            }
        }
        
        // Build tree
        double Md = 15; // batch size
        double bd = 2; // branching factor
        double n_train_d = 1000; // training patch
        double Ld = 5; // number of layer
        int ndim = 1;
        int dim[2] = {width, height};

        int treeDim[2];
        double * tree =  build_tree( &I[0], &Md, &bd, &n_train_d, &Ld, ndim, dim, treeDim);
        
//        printf("Build tree, dim %d x %d \n", treeDim[0], treeDim[1]);
        
        // Build matrix A; same dimension with image
        double * A = search_tree(&I[0], tree, &bd, ndim, 2, dim, treeDim);
        
        // Matrix B
        vector<ij> Bij = biadjacency_matrix(A, width, height, Md);
        
//        printf("Build matrix sparse size: %lu \n", Bij.size());
        
        

        
        
        // Using armadillo to compute the matrix
        long B_width = width*height;
        int K = 0;
        for (int a=0; a<width*height; a++)
            if (A[a]>K)
                K = A[a];
        long B_height = Md*Md*K;
        
        Mat<double> B(B_width, B_height);
        Mat<double> Bt(B_height, B_width);
        
        vec B1 = zeros(B.n_rows);
        vec Bt1 = zeros(B.n_rows);

        for (auto & ij : Bij)
        {
            B(ij.i, ij.j) = 1;
            B1(ij.i) ++;
            
            Bt(ij.j, ij.i) = 1;
            Bt1(ij.j)++;
        }

       

        
        // Mapping matrix
        printf("Compute mapping matrix \n");
        


        

        printf("Mapping matrix \n");
        
        Mat<double> T1 = zeros(Bt.n_rows, Bt.n_cols);
        
        for(auto ij : Bij)
        {
            T1(ij.j, ij.i) = Bt(ij.j, ij.i)/(Bt1(ij.j) + 0.00001);
        }
        
        Mat<double> T2 = zeros(B.n_rows, B.n_cols);
        
        for(auto ij : Bij)
        {
            T2(ij.i, ij.j) = B(ij.i, ij.j)/(B1(ij.i) + 0.00001);
        }
        
        

        
        printf("Label \n");
        // test labelling
        vec label1 = ones(width*height)*0.5;
        vec label2 = ones(width*height)*0.5;
        
        for (int i = width/2 - width/10; i < width/2 + width/10; i++)
        {
            for (int j = height/2 - height/10; j < height/2 + height/10; j++)
            {
                label1(j*width + i) = 1;
                label2(j*width + i) = 0;
            }
        }
        
        for (int i = 3*width/4; i < 3*width/4 + width/6; i++)
        {
            for (int j = 3*height/4; j < 3*height/4 + height/6; j++)
            {
                label1(j*width + i) = 0;
                label2(j*width + i) = 1;
            }
        }
        
        t1.done();
        
        profile t2("multiply");

        
        printf("computing probability \n");
        vec p1 = T2*(T1*label1);
        vec p2 = T2*(T1*label2);
        
        Mat<double> out = zeros(width*height);
        for (int i = 0; i < height; i++)
        {
            for(int j = 0; j < width; j++)
            {
                int idx = i*width + j;
                out(idx) = (p1(idx) > p2(idx))? 1:0;
            }
        }
        
        {
            imaged test(label1.memptr(), width, height);
            test.display("label");
        }

        {
            imaged test(out.memptr(), width, height);
            test.display("gray");
        }

//        {
//            imaged image(p1.memptr(), width, height);
//            image.normalize(0, 255);
//            image.save("file.bmp");
//            
//            image.display("Probability");
//        }
    }
}

















































