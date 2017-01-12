//
//  LemonScriptSymbolTableStack.hpp
//  lemonscript
//
//  Created by Donald Pinckney on 2/6/16.
//  Copyright © 2016 Donald Pinckney. All rights reserved.
//

#ifndef LemonScriptSymbolTableStack_hpp
#define LemonScriptSymbolTableStack_hpp

#include <stdio.h>
#include <string>
#include <vector>
#include "AvailableCppCommandDeclaration.h"
#include "LemonScriptSymbolTable.h"

class LemonScriptSymbolTableStack {
    
    // End of the array is the top of the stack
    std::vector<LemonScriptSymbolTable> stack;
    
public:
    
    LemonScriptSymbolTableStack() : stack(1, LemonScriptSymbolTable()) { }
    
    // Declaration function
    void declareVariable(int line, const std::string &name, lemonscript::DataType type, void *pointerToValue);
    void declareGlobalVariable(int line, const std::string &name, lemonscript::DataType type, void *pointerToValue);
    
    // Stack manipulations
    void pushScope();
    void popScope();
    
    // Lookup functions
    // Returns NULL if variableName is not in the symbol table
    void *addressOfVariable(const std::string &variableName) const;
    
    // Undefined what is returned if variableName does not exist
    lemonscript::DataType typeOfVariable(const std::string &variableName) const;    
};

#endif /* LemonScriptSymbolTableStack_hpp */
