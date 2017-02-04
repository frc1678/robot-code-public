//
//  CppCommand.hpp
//  lemonscript
//
//  Created by Donald Pinckney on 12/24/15.
//  Copyright © 2015 Donald Pinckney. All rights reserved.
//

#ifndef CppCommand_hpp
#define CppCommand_hpp

#include <stdio.h>
#include <vector>
#include <memory>
#include "Command.h"
#include "AvailableCppCommandDeclaration.h"
#include "BaseAutoFunction.h"

#include "lemonscript.h"

class lemonscript::CppCommand : public Command {
    
    const AvailableCppCommandDeclaration *declaration = nullptr;
    std::unique_ptr<BaseAutoFunction> autoFunc = nullptr;
    
    
    std::vector<bool> isArgumentLiteral;
    std::vector<void *> parameterValues; // parallel to `parameters` in declaration (left to right order). If a parameter isConstant, then this contains the constant value directly.  If a parameter is an expression, then this contains a pointer to the expression
    
public:
    CppCommand(int l, LemonScriptState *s, const std::string &commandString);
    virtual ~CppCommand();
    
    bool Update();
    
private:
    void allocateAutoFunction(std::vector<void *> args);
    
};

#endif /* CppCommand_hpp */
