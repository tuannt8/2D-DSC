//
//  sparse_mat.h
//  DSC_texture_seg
//
//  Created by Tuan Nguyen Trung on 4/20/16.
//  Copyright © 2016 Asger Nyman Christiansen. All rights reserved.
//

#ifndef sparse_mat_h
#define sparse_mat_h

#include <vector>
#include <armadillo>

struct ij
{
    long i,j;
    ij(){i = -1; j = -1;}
    ij(long i_, long j_) : i(i_), j(j_){}; // constructor
//    bool operator<(const ij second) const{ // leq operator needed for sorting
//        return (j<second.j) || ((j==second.j) && (i<second.i));
//    }
    
    bool operator<(const ij second) const{ // leq operator needed for sorting
        return (i<second.i) || ((i==second.i) && (j<second.j));
    }
};

#define NUM_THREADS 4
class sparse_mat
{
public:
    sparse_mat(){};
    ~sparse_mat(){};
    
    sparse_mat(const std::vector<ij> & Bij, long nrow, long ncol)
    {
        _n_row = nrow;
        _n_col = ncol;
        
        long nb_thread_elem = std::ceil(_n_row / NUM_THREADS);
        
        long tidx = 0;
        for(auto ije : Bij)
        {
            long r = ije.i;
            
            _indices[tidx].push_back(ije);
            
            tidx = r/nb_thread_elem;
        }
    };
    

private:
    long _n_row, _n_col;
    std::vector<ij> _indices[NUM_THREADS];
    std::vector<double> _values;
};

#endif /* sparse_mat_h */
