//
//  CppCommand.cpp
//  FiniteStateMachine
//
//  Created by Donald Pinckney on 12/24/15.
//  Copyright © 2015 Donald Pinckney. All rights reserved.
//

#include "CppCommand.h"
#include "Expression.h"

#include <strings.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>

#include "ParsingUtils.h"

using namespace std;

using namespace lemonscript_expressions;


inline bool IsSpace (char c) { return c == ' '; }

lemonscript::CppCommand::CppCommand(int l, LemonScriptState *state, const std::string &commandStringInput) : Command(l, state) {
    std::string commandString = ParsingUtils::removeCommentFromLine(commandStringInput);
    
    string functionName;
    vector<string> argumentStrings;
    
    size_t colonIndex = commandString.find(":");
    functionName = commandString.substr(0, colonIndex);
    functionName = ParsingUtils::trimWhitespace(functionName);
    
    if(colonIndex != string::npos) { // There are arguments
        string argumentsString = commandString.substr(colonIndex + 1);
        
        // Remove whitespace in the string, make the parsing a bit easier
        argumentsString.erase(std::remove_if(argumentsString.begin(), argumentsString.end(), IsSpace), argumentsString.end());
        
        auto endOfArgIt = argumentsString.rbegin();

        for (auto it = argumentsString.rbegin(); it != argumentsString.rend(); ++it) {
            
            char c = *it;
            
            auto startOfArgIt = it;
            auto newEndOfArgIt = it;
            if(c == ':') { // Then we want to eat up the prefix to the argument
                while (true) {
                    char d = *it;
                    if(d == ',' || it + 1 == argumentsString.rend()) {
                        newEndOfArgIt = it + 1;
                        break;
                    }
                    ++it;
                }
            }
            
            if(c == ':' || c == ',' || it + 1 == argumentsString.rend()) { // Then we reached the prefix before the argument expression
                if(it == argumentsString.rend()) {
                    //break;
                }
                
                if(startOfArgIt + 1 == argumentsString.rend()) {
                    startOfArgIt = argumentsString.rend();
                }
                
                if(c == ',') {
                    ++newEndOfArgIt;
                }
                
                string argumentString(endOfArgIt, startOfArgIt);
                reverse(argumentString.begin(), argumentString.end());
                                
                endOfArgIt = newEndOfArgIt;
                
                argumentStrings.push_back(argumentString);
            }
        }
    }
    
    reverse(argumentStrings.begin(), argumentStrings.end());
    
    // Now that we have all the expression strings parsed, test all different command overrides.
    vector<const AvailableCppCommandDeclaration *> overrides = state->lookupCommandDeclarationsForName(functionName);
    if(overrides.size() == 0) {
        throw "Line " + to_string(l) + ": No command named: " + functionName;
    }

    
    int matchCount = 0;
    const AvailableCppCommandDeclaration *matchedDecl = NULL;
    for (auto it = overrides.begin(); it != overrides.end(); ++it) {
        const AvailableCppCommandDeclaration *decl = *it;
        
        if(decl->parameters.size() != argumentStrings.size()) {
            continue;
        }
        
        
        bool matched = true;
        bool perfectMatch = true;
        for (size_t paramIndex = 0; paramIndex < decl->parameters.size(); paramIndex++) {
            DataType paramType = decl->parameters[paramIndex];
            string argString = argumentStrings[paramIndex];
            
            try {
                Expression expr(argString, paramType, state, l);
                if(expr.neededToSupertype()) {
                    perfectMatch = false;
                }
            } catch (string err) {
                matched = false;
                break;
            }
        }
        
        
        if(!matched) {
            continue;
        }
        
        
        matchCount++;
        matchedDecl = decl;
        
        if(perfectMatch) {
            matchCount = 1;
            break;
        }
    }
    
    if(matchCount == 0) {
        throw "Line " + to_string(l) + ": Could not match types for: " + functionName;
    } else if(matchCount >= 2) {
        throw "Line " + to_string(l) + ": Could not disambiguate types for: " + functionName;
    }
    
    
    
    declaration = matchedDecl;
    
    for (size_t paramIndex = 0; paramIndex < declaration->parameters.size(); paramIndex++) {
        string argString = argumentStrings[paramIndex];
        DataType paramType = declaration->parameters[paramIndex];
        
        Expression *expr = new Expression(argString, paramType, state, l);
        if(expr->isConstant()) {
            if(paramType == DataType::INT) {
                int val;
                expr->getValue(&val);
                parameterValues.push_back(reinterpret_cast<void *>(val));
            } else if(paramType == DataType::FLOAT) {
                float val;
                expr->getValue(&val);
                int tempArgVal = *((int *)&val);
                parameterValues.push_back(reinterpret_cast<void *>(tempArgVal));
            } else if(paramType == DataType::BOOLEAN) {
                bool val;
                expr->getValue(&val);
                parameterValues.push_back((void *)val);
            }
            
            isArgumentLiteral.push_back(true);
            
            delete expr;
            
        } else {
            isArgumentLiteral.push_back(false);
            parameterValues.push_back(expr);
        }
    }
}

lemonscript::CppCommand::~CppCommand() {
    for (size_t i = 0; i < parameterValues.size(); i++) {
        if(isArgumentLiteral[i] == false) {
            lemonscript_expressions::Expression *expr = (lemonscript_expressions::Expression *)parameterValues[i];
            delete expr;
        }
    }
}


void lemonscript::CppCommand::allocateAutoFunction(vector<void *> args) {
    autoFunc = declaration->generatorFunction();
    autoFunc->Init(args);
}
    
bool lemonscript::CppCommand::Update() {
    void *data = savedState->userData;
    
    vector<void *> arguments = {data};
    
    int *argumentEvaluation = new int[parameterValues.size()];
    bzero(argumentEvaluation, sizeof(int) * parameterValues.size());
    
    for(size_t i = 0; i < parameterValues.size(); i++) {
        if(isArgumentLiteral[i]) {
            argumentEvaluation[i] = *((int *)&parameterValues[i]);
        } else {
            ((Expression *)parameterValues[i])->getValue(argumentEvaluation + i);
        }
        
        arguments.push_back(argumentEvaluation + i);
    }
    
    if(autoFunc == NULL) {
        allocateAutoFunction(arguments);
    }
    
    bool retVal = autoFunc->Periodic(arguments);
    delete [] argumentEvaluation;
    return retVal;
}
