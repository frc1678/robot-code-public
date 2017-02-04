//
//  BaseAutoFunction.h
//  lemonscript
//
//  Created by Donald Pinckney on 5/25/16.
//  Copyright © 2016 Donald Pinckney. All rights reserved.
//

#ifndef BaseAutoFunction_h
#define BaseAutoFunction_h

#include <stdio.h>

namespace lemonscript {
    class BaseAutoFunction {
    public:
        virtual bool Init(std::vector<void *>) = 0;
        virtual bool Periodic(std::vector<void *>) = 0;
        
    private:
        //none
    };
}


#endif /* BaseAutoFunction_h */
