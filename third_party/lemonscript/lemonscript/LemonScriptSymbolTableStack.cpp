//
//  LemonScriptSymbolTableStack.cpp
//  lemonscript
//
//  Created by Donald Pinckney on 2/6/16.
//  Copyright © 2016 Donald Pinckney. All rights reserved.
//

#include "LemonScriptSymbolTableStack.h"


// Declaration function
void LemonScriptSymbolTableStack::declareVariable(int line, const std::string &name, lemonscript::DataType type, void *pointerToValue) {
    stack[stack.size() - 1].declareVariable(line, name, type, pointerToValue);
}
void LemonScriptSymbolTableStack::declareGlobalVariable(int line, const std::string &name, lemonscript::DataType type, void *pointerToValue) {
    stack[0].declareVariable(line, name, type, pointerToValue);
}

// Stack manipulations
void LemonScriptSymbolTableStack::pushScope() {
    stack.push_back(LemonScriptSymbolTable());
}
void LemonScriptSymbolTableStack::popScope() {
    stack.pop_back();
}

// Lookup functions
// Returns NULL if variableName is not in the symbol table
void * LemonScriptSymbolTableStack::addressOfVariable(const std::string &variableName) const {
    for (auto it = stack.crbegin(); it != stack.crend(); it++) {
        void *address = it->addressOfVariable(variableName);
        if(address != NULL) {
            return address;
        }
    }
    return NULL;
}

// Undefined what is returned if variableName does not exist
lemonscript::DataType LemonScriptSymbolTableStack::typeOfVariable(const std::string &variableName) const {
    for (auto it = stack.crbegin(); it != stack.crend(); it++) {
        void *address = it->addressOfVariable(variableName);
        if(address != NULL) {
            return it->typeOfVariable(variableName);
        }
    }
    return lemonscript::DataType::INT;
}
