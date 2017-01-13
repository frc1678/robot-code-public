//
//  ImportCommand.hpp
//  lemonscript
//
//  Created by Donald Pinckney on 2/21/16.
//  Copyright © 2016 Donald Pinckney. All rights reserved.
//

#ifndef ImportCommand_hpp
#define ImportCommand_hpp

#include <stdio.h>

#include "SequentialCommand.h"

#include "lemonscript.h"

class lemonscript::ImportCommand : public Command {
    
    LemonScriptCompiler *importCompiler = NULL;
    
public:
    ImportCommand(int l, LemonScriptState *s, const std::string &commandString);
    virtual ~ImportCommand();
    
    bool Update();
    
};

#endif /* ImportCommand_hpp */
