//
//  LemonScriptState.hpp
//  lemonscript
//
//  Created by Donald Pinckney on 12/24/15.
//  Copyright © 2015 Donald Pinckney. All rights reserved.
//

#ifndef LemonScriptState_hpp
#define LemonScriptState_hpp

#include <stdio.h>
#include <string>
#include <vector>
#include <map>

#include "AvailableCppCommandDeclaration.h"
#include "LemonScriptSymbolTableStack.h"

#include "lemonscript.h"

class lemonscript::LemonScriptState {
    
    LemonScriptSymbolTableStack symbolStack;
    std::map<std::string, std::vector<const AvailableCppCommandDeclaration *> > availableCppCommands;
    std::string searchPath = "";
    std::vector<std::string> imports;
    std::vector<std::string> evaluatedImports;
    
public:
    ~LemonScriptState();
    
    void *userData = NULL;
    
    // Variable declaration functions
    void declareGlobalVariable(int line, const std::string &name, DataType type, void *pointerToValue);
    void declareVariable(int line, const std::string &name, DataType type, void *pointerToValue);
    
    // Symbol stack methods
    void pushScope();
    void popScope();
    
    // Please use these 2 methods sparingly...
    LemonScriptSymbolTableStack getScope(); // Warning, don't modify the returned stack.
    void restoreScope(const LemonScriptSymbolTableStack &scope);
    
    
    // Command declaration functions
    void declareAvailableCppCommand(const AvailableCppCommandDeclaration *decl);
    void declareAvailableCppCommands(const std::vector<const AvailableCppCommandDeclaration *> decls);
    
    // Lookup functions
    void *addressOfVariable(const std::string &variableName) const;
    
    // Undefined what is returned if variableName does not exist
    DataType typeOfVariable(const std::string &variableName) const;
    
    const AvailableCppCommandDeclaration *lookupCommandDeclaration(const std::string &name, const std::vector<DataType> &parameterTypes) const;
    std::vector<const AvailableCppCommandDeclaration *> lookupCommandDeclarationsForName(const std::string &name) const;
    
    // Search path settings and IMPORT tweaks
    void setSearchPath(const std::string &searchPath);
    std::string getSearchPath() const;
    
    void setIMPORTs(const std::vector<std::string> &imports);
    std::vector<std::string> getIMPORTs() const;
    
    void addEvaluatedIMPORT(const std::string &name);
    bool alreadyEvaluatedIMPORT(const std::string &name) const;
};

#endif /* LemonScriptState_hpp */
