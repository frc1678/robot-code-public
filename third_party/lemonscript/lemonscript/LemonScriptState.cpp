//
//  LemonScriptState.cpp
//  lemonscript
//
//  Created by Donald Pinckney on 12/24/15.
//  Copyright © 2015 Donald Pinckney. All rights reserved.
//

#include <stdlib.h>
#include <string.h>
#include <algorithm>

#include "LemonScriptState.h"

#include "ParsingUtils.h"

lemonscript::LemonScriptState::~LemonScriptState() {
    LemonScriptSymbolTable::freeVariables();
    
    for(auto mapIt : availableCppCommands) {
        for(auto vectorIt = mapIt.second.begin(); vectorIt != mapIt.second.end(); vectorIt++) {
            delete *vectorIt;
        }
    }
}

void lemonscript::LemonScriptState::declareGlobalVariable(int line, const std::string &name, DataType type, void *pointerToValue) {
    symbolStack.declareGlobalVariable(line, name, type, pointerToValue);
}

void lemonscript::LemonScriptState::declareVariable(int line, const std::string &name, DataType type, void *pointerToValue) {
    symbolStack.declareVariable(line, name, type, pointerToValue);
}

void lemonscript::LemonScriptState::pushScope() {
    symbolStack.pushScope();
}

void lemonscript::LemonScriptState::popScope() {
    symbolStack.popScope();
}

LemonScriptSymbolTableStack lemonscript::LemonScriptState::getScope() {
    return symbolStack;
}

void lemonscript::LemonScriptState::restoreScope(const LemonScriptSymbolTableStack &scope) {
    symbolStack = scope;
}


void lemonscript::LemonScriptState::declareAvailableCppCommand(const AvailableCppCommandDeclaration *decl) {
    std::string name = decl->functionName;
    std::vector<const AvailableCppCommandDeclaration *> &commands = availableCppCommands[name];
    for (auto it = commands.begin(); it != commands.end(); ++it) {
        if((*it)->parameters == decl->parameters) {
            throw "Duplicate command definition with params: " + name;
        }
    }
    
    commands.push_back(decl);
}

void lemonscript::LemonScriptState::declareAvailableCppCommands(const std::vector<const AvailableCppCommandDeclaration *> decls) {
    // Declare available commands
    for (auto it = decls.begin(); it != decls.end(); ++it) {
        declareAvailableCppCommand(*it);
    }
}

void * lemonscript::LemonScriptState::addressOfVariable(const std::string &variableName) const {
    return symbolStack.addressOfVariable(variableName);
}

lemonscript::DataType lemonscript::LemonScriptState::typeOfVariable(const std::string &variableName) const {
    return symbolStack.typeOfVariable(variableName);
}

const lemonscript::AvailableCppCommandDeclaration * lemonscript::LemonScriptState::lookupCommandDeclaration(const std::string &name, const std::vector<DataType> &parameterTypes) const {
    auto mapIt = availableCppCommands.find(name);
    if(mapIt == availableCppCommands.end()) {
        return NULL;
    }
    
    
    const std::vector<const AvailableCppCommandDeclaration *> &commands = mapIt->second;
    for (auto it = commands.begin(); it != commands.end(); ++it) {
        const AvailableCppCommandDeclaration *decl = *it;
        if(decl->parameters == parameterTypes) {
            return decl;
        }
    }
    
    return NULL;
}

std::vector<const lemonscript::AvailableCppCommandDeclaration *> lemonscript::LemonScriptState::lookupCommandDeclarationsForName(const std::string &name) const {
    auto mapIt = availableCppCommands.find(name);
    if(mapIt == availableCppCommands.end()) {
        return {};
    }
    
    return mapIt->second;
}

void lemonscript::LemonScriptState::setSearchPath(const std::string &searchPath) {
    if(searchPath != "" && searchPath[searchPath.length() - 1] != '/') {
        this->searchPath = searchPath + "/";
    } else {
        this->searchPath = searchPath;
    }
}

std::string lemonscript::LemonScriptState::getSearchPath() const {
    return searchPath;
}

void lemonscript::LemonScriptState::setIMPORTs(const std::vector<std::string> &imports) {
    this->imports = imports;
}

std::vector<std::string> lemonscript::LemonScriptState::getIMPORTs() const {
    return imports;
}

void lemonscript::LemonScriptState::addEvaluatedIMPORT(const std::string &name) {
    evaluatedImports.push_back(name);
}

bool lemonscript::LemonScriptState::alreadyEvaluatedIMPORT(const std::string &name) const {
    return std::find(evaluatedImports.begin(), evaluatedImports.end(), name) != evaluatedImports.end();
}

