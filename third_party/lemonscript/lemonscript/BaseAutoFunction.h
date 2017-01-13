//
//  asdf.hpp
//  lemonscript
//
//  Created by Donald Pinckney on 5/25/16.
//  Copyright © 2016 Donald Pinckney. All rights reserved.
//

#ifndef asdf_hpp
#define asdf_hpp

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


#endif /* asdf_hpp */
