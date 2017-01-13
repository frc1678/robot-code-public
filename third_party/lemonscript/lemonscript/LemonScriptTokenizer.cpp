//
//  LemonScriptParser.cpp
//  FiniteStateMachine
//
//  Created by Donald Pinckney on 12/30/15.
//  Copyright © 2015 Donald Pinckney. All rights reserved.
//

#include "LemonScriptTokenizer.h"

#include <tuple>

#include "ParsingUtils.h"

using namespace std;
using namespace ParsingUtils;


bool isEof(std::istream *i) {
  return i->eof() || i->tellg() == -1;
}

tuple<string, TokenType, int> lemonscript::LemonScriptTokenizer::nextToken() {
    
    string firstLine;
    getline(*input, firstLine);
    if(isEof(input) && firstLine.length() == 0) {
        return make_tuple("", NOT_A_TOKEN, -1);
    }
    
    currentLine++;
    
    while(isExecutableLine(firstLine) == false) {
        getline(*input, firstLine);
        if(isEof(input) && firstLine.length() == 0) {
            return make_tuple("", NOT_A_TOKEN, -1);
        }
        
        currentLine++;
    }
    
    
    // Now we have an executable line in firstLine
    // We now categorize firstLine
    int startLine = currentLine;
    bool isScoping = false;
    TokenType type;
    if(beginsWith(firstLine, "WHILE:")) {
        isScoping = true;
        type = WhileAlsoToken;
    } else if(beginsWith(firstLine, "SET ")) {
        type = SetToken;
    } else if(beginsWith(firstLine, "DEF ")) {
        type = DefToken;
    } else if(beginsWith(firstLine, "IMPORT ")) {
        type = ImportToken;
    } else if(beginsWith(firstLine, "RUN ")) {
        type = RunToken;
    } else if(beginsWith(firstLine, "COMPLETE:")) {
        isScoping = true;
        type = CompleteToken;
    } else if(beginsWith(firstLine, "COMPLETE ANY:")) {
        isScoping = true;
        type = CompleteAnyToken;
    } else if(beginsWith(firstLine, "IF:")) {
        isScoping = true;
        type = IfElseIfElseToken;
    } else if(beginsWith(firstLine, "NOT ")) {
        type = NotToken;
    } else if(beginsWith(firstLine, "? ")) {
        type = OptionalCommandToken;
    } else {
        type = CppToken;
    }
    
    
    // Now we need to grab the following lines that belong
    ostringstream tempTokenStorage;
    if(!isScoping) {
        tempTokenStorage << firstLine;
    } else if(type == WhileAlsoToken) {
        while(true) {
            tempTokenStorage << firstLine << endl;
        

            string nextLine = peekLine(*input);
            if((!beginsWith(nextLine, "  ") && !beginsWith(nextLine, "ALSO:") && isExecutableLine(nextLine)) || isEof(input)) {
                break;
            }

            getline(*input, firstLine);
            currentLine++;
            
            
        }
    } else if(type == CompleteToken || type == CompleteAnyToken) {
        while(true) {
            tempTokenStorage << firstLine << endl;
            
            string nextLine = peekLine(*input);
            if((!beginsWith(nextLine, "  ") && isExecutableLine(nextLine)) || isEof(input)) {
                break;
            }
            
            getline(*input, firstLine);
            currentLine++;
        }
    } else if(type == IfElseIfElseToken) {
        while(true) {
            tempTokenStorage << firstLine << endl;
            
            string nextLine = peekLine(*input);
            if((!beginsWith(nextLine, "  ") && !beginsWith(nextLine, "THEN:") && !beginsWith(nextLine, "ELSE IF:") && !beginsWith(nextLine, "ELSE:") && isExecutableLine(nextLine)) || isEof(input)) {
                break;
            }
            
            
            getline(*input, firstLine);
            currentLine++;
        }
    }
    
    return make_tuple(tempTokenStorage.str(), type, startLine);
    
}
