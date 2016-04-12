//
//  texture.cpp
//  DSC_texture_seg
//
//  Created by Tuan Nguyen Trung on 08/03/16.
//  Copyright © 2016 Asger Nyman Christiansen. All rights reserved.
//

#include <stdio.h>
#include "texture.h"

#include "define.h"

#include "profile.h"

namespace texture
{
    using namespace std;
    using namespace cimg_library;
    using namespace arma;
    
    dictionary::dictionary(std::string imName)
    {
//        if (load_up(imName))
//        {
//            std::cout << "Dictionary loaded at " << get_back_up_name(imName) << std::endl;
//            return;
//        }
        
        std::cout << "Building dictionary" << std::endl;
        
        // Load test image
        imageb im;
        im.load(imName.data());
        
        int width = im.width();
        int height = im.height();
       
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
        
        std::cout << "Building tree" << std::endl;
        // Build tree
        double Md = 15;             // batch size
        double bd = 2;              // branching factor
        double n_train_d = 1000;    // training patch
        double Ld = 5;              // number of layer
        int ndim = 1;
        int dim[2] = {width, height};
        
        int treeDim[2];
        double * tree =  build_tree( &I[0], &Md, &bd, &n_train_d, &Ld, ndim, dim, treeDim);
        
        std::cout << "Building adjacency matrix" << std::endl;
        // Build matrix A; same dimension with image
        double * A = search_tree(&I[0], tree, &bd, ndim, 2, dim, treeDim);
        
        // Matrix B
        vector<ij> Bij = biadjacency_matrix(A, width, height, Md);
        
        
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
        
        std::cout << "Compute T1 and T2" << std::endl;
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
    
        
        _T1 = T1;
        _T2 = T2;
        
        //        back_up(imName);
        std::cout << "Dictionary built" << std::endl;
    }
    
    void dictionary::save_matrix(std::string imName)
    {
        std::ostringstream mat_name1, mat_name2;
        mat_name1 << imName << "_1.txt";
        mat_name2 << imName << "_2.txt";
        _T1.save(mat_name1.str().data(), arma::arma_ascii);
        _T2.save(mat_name2.str().data(), arma::arma_ascii);
    }
    
    bool dictionary::load_matrix(std::string imName)
    {
        std::ostringstream mat_name1, mat_name2;
        mat_name1 << imName << "_1.txt";
        mat_name2 << imName << "_2.txt";
        std::ifstream f1(mat_name1.str());
        std::ifstream f2(mat_name2.str());
        if (f1.good() and f2.good())
        {
            _T1.load(mat_name1.str().data(), arma::arma_ascii);
            _T2.load(mat_name2.str().data(), arma::arma_ascii);
            return true;
        }
        else
            return false;
    }
#ifdef __APPLE__
#ifndef st_mtime
#define st_mtime st_mtimespec.tv_sec
#endif
#endif
    
    std::string dictionary::get_back_up_name(const std::string file) {
        struct tm *clock;
        struct stat attr;
        
        stat(file.c_str(), &attr);
        clock = gmtime(&(attr.st_mtime));
        
        auto name = file.substr(file.find_last_of("/\\") + 1);
        
        std::ostringstream os;
        os << "DATA/dictionary_backup/" << name << "_" << clock->tm_year << "_" << clock->tm_mon << "_"
            << clock->tm_mday << "_" << clock->tm_hour << "_"
            << clock->tm_min  << "_" << clock->tm_sec ;
        
        return os.str();
    }
    
    bool dictionary::load_up(std::string imName)
    {
        auto name = get_back_up_name(imName);
        return load_matrix(name);
    }
    
    void dictionary::back_up(std::string imName, bool overwrite)
    {
        auto name = get_back_up_name(imName);
        // Load the info file
        std::ifstream ifile(name);
        if (!overwrite and ifile.good())
        {
            ifile.close();
            // Existed
        }
        else
        {
            save_matrix(name);
            std::cout << "Dictionary saved at " << name << std::endl;
        }
    }
    
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

















































