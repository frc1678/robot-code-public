//
//  PlayTestsShared.hpp
//  lemonscript
//
//  Created by Donald Pinckney on 5/23/16.
//  Copyright © 2016 Donald Pinckney. All rights reserved.
//

#ifndef PlayTestsShared_hpp
#define PlayTestsShared_hpp

#include <lemonscript/lemonscript.h>
#include <string>

namespace PlayTestsShared {
    lemonscript::LemonScriptState * play_tests_make_state();
    void run_at_path(std::string path, lemonscript::LemonScriptState *state);

}



#endif /* PlayTestsShared_hpp */
