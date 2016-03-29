//
//  texture.cpp
//  DSC_texture_seg
//
//  Created by Tuan Nguyen Trung on 08/03/16.
//  Copyright © 2016 Asger Nyman Christiansen. All rights reserved.
//

#include <stdio.h>
#include "texture.h"
#include <armadillo>
#include "CImg.h"
#include <string>
#include "define.h"

#include "profile.h"

namespace texture
{
    using namespace std;
    using namespace cimg_library;
    using namespace arma;
    
    typedef unsigned char BYTE;
    #define MAX_BYTE 255
    
    typedef CImg<BYTE> imageb;
    typedef CImg<double> imaged;
    
    void test()
    {

        profile t1("Begining to bij");
    
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
        

        
//        for (int i = 0; i < B1.n_elem; i++)
//        {
//            B1(i) = 1 / (B1(i) + 0.00000001);
//        }
//        
//        for (int i = 0; i < Bt1.n_elem; i++)
//        {
//            Bt1(i) = 1 / (Bt1(i) + 0.00000001);
//        }
        
//        vec one = ones<vec>(Bt.n_cols);
//        
//        vec Bt1 = Bt*one;
//        Mat<double> Bt1_i = zeros(Bt1.n_elem, Bt1.n_elem);
//        for (int i = 0; i < Bt1.n_elem; i++)
//        {
//            Bt1_i(i,i) = 1./(Bt1(i) + 0.00000001);
//        }
//        
//        
//        vec one_h = ones<vec>(B.n_cols);
//        vec B1 = B*one_h;
//        Mat<double> B1_i = zeros(B1.n_elem, B1.n_elem);
//        for(int i = 0; i < B1.n_elem; i++)
//        {
//            B1_i(i,i) = 1./(B1(i) + 0.00000001);
//        }
        

        printf("Mapping matrix \n");
        
        Mat<double> T1 = zeros(Bt.n_rows, Bt.n_cols);
        
        for(auto ij : Bij)
        {
            T1(ij.j, ij.i) = Bt(ij.j, ij.i)/(Bt1(ij.j) + 0.00001);
        }
        
//        for (int i = 0; i < Bt.n_rows; i++)
//        {
//            for (int j = 0; j < Bt.n_cols; j++)
//            {
//                T1(i,j) = Bt(i,j)/(Bt1(i) + 0.00001);
//            }
//        }
        
        Mat<double> T2 = zeros(B.n_rows, B.n_cols);
        
        for(auto ij : Bij)
        {
            T2(ij.i, ij.j) = B(ij.i, ij.j)/(B1(ij.i) + 0.00001);
        }
        
//        for (int i = 0; i < B.n_rows; i++)
//        {
//            for (int j = 0; j < B.n_cols; j++)
//            {
//                T2(i,j) = B(i,j)/(B1(i) + 0.00001);
//            }
//        }
        

        

        
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