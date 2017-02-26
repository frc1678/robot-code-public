//
//  ImportCommand.cpp
//  lemonscript
//
//  Created by Donald Pinckney on 2/21/16.
//  Copyright © 2016 Donald Pinckney. All rights reserved.
//

#include "ImportCommand.h"

#include "LemonScriptCompiler.h"
#include "ParsingUtils.h"

lemonscript::ImportCommand::ImportCommand(int l, LemonScriptState *s, const std::string &commandString) : Command(l, s) {

    const std::string importDelim = "IMPORT";
    size_t importLoc = commandString.find(importDelim);
    size_t endOfImportLoc = importLoc + importDelim.length();
    
    std::string importFileName = commandString.substr(endOfImportLoc);
    importFileName = ParsingUtils::trimWhitespace(importFileName);
    
    if(s->alreadyEvaluatedIMPORT(importFileName)) {
        importCompiler = nullptr;
        _hasExternalCode = false;
    } else {
        s->addEvaluatedIMPORT(importFileName);
        importCompiler = new lemonscript::LemonScriptCompiler(importFileName, s);
        _hasExternalCode = importCompiler->RootSequence()->HasExternalCode();
    }
}

lemonscript::ImportCommand::~ImportCommand() {
    if(importCompiler != nullptr) {
        delete importCompiler;
    }
}


bool lemonscript::ImportCommand::Update() {
    if(importCompiler == nullptr) {
        return true;
    }
    return importCompiler->PeriodicUpdate();
}

bool lemonscript::ImportCommand::fastForward() {
    return !HasExternalCode();
}
