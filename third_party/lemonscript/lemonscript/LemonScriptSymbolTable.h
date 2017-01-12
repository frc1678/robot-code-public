//
//  LemonScriptSymbolTable.hpp
//  lemonscript
//
//  Created by Donald Pinckney on 2/6/16.
//  Copyright © 2016 Donald Pinckney. All rights reserved.
//

#ifndef LemonScriptSymbolTable_hpp
#define LemonScriptSymbolTable_hpp

#include <stdio.h>
#include <string>
#include <map>
#include "AvailableCppCommandDeclaration.h"

class LemonScriptSymbolTable {
    
    // Will contain things such as variable bindings, etc.
    std::map<std::string, void *> variableAddresses;
    std::map<std::string, lemonscript::DataType> variableTypes;
    
    static std::vector<void *> variableAddressHeap;
    
public:
    // Declaration function
    void declareVariable(int line, const std::string &name, lemonscript::DataType type, void *pointerToValue);

    
    // Lookup functions
    // Returns NULL if variableName is not in the symbol table
    void *addressOfVariable(const std::string &variableName) const;
    
    // Undefined what is returned if variableName does not exist
    lemonscript::DataType typeOfVariable(const std::string &variableName) const;
    
    static void freeVariables();
};

#endif /* LemonScriptSymbolTable_hpp */
