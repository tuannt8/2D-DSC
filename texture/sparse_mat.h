//
//  sparse_mat.h
//  DSC_texture_seg
//
//  Created by Tuan Nguyen Trung on 4/20/16.
//  Copyright © 2016 Asger Nyman Christiansen. All rights reserved.
//

#ifndef sparse_mat_h
#define sparse_mat_h

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

class sparse_mat
{
public:
    sparse_mat(){};
    ~sparse_mat(){};
    
    
public:
    long _n_row, n_col;
    std::vector<ij> _indices;
    std::vector<double> _values;
};

#endif /* sparse_mat_h */
