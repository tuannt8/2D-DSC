//
//  interface.h
//  DSC
//
//  Created by Tuan Nguyen Trung on 2/9/15.
//  Copyright (c) 2015 Asger Nyman Christiansen. All rights reserved.
//

#pragma once

#include "user_interface.h"

class interface{
public:
    interface(){};
    interface(const int &argc, const char** argv);
    virtual ~interface(){};
    
private:
};