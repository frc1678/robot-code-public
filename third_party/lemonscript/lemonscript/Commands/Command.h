//
//  Command.hpp
//  lemonscript
//
//  Created by Donald Pinckney on 12/24/15.
//  Copyright © 2015 Donald Pinckney. All rights reserved.
//

#ifndef Command_hpp
#define Command_hpp

#include <stdio.h>
#include <string>
#include <vector>

#include "LemonScriptState.h"

#include "lemonscript.h"

class lemonscript::Command {

protected:
    // This needs to be set to true if the command runs code external to lemonscript. For example, a CppCommand.
    // True prevents the command from being sequentially optimized.
    bool _hasExternalCode = true;

public:
    LemonScriptState *savedState;
    
    Command(int l, LemonScriptState *s) : savedState(s), lineNumber(l) { };
    virtual ~Command() { }
    
    int lineNumber;
    
    virtual bool Update() = 0;
    virtual bool fastForward() = 0;
    bool HasExternalCode() const { return _hasExternalCode; };
};

#endif /* Command_hpp */
