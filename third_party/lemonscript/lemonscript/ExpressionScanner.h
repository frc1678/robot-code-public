//
//  ExpressionScanner.hpp
//  lemonscript
//
//  Created by Donald Pinckney on 2/22/16.
//  Copyright © 2016 Donald Pinckney. All rights reserved.
//

#ifndef ExpressionScanner_hpp
#define ExpressionScanner_hpp

#include <stdio.h>
#include <string>
#include <sstream>

#include "expressions.h"

    // "(", ",", ")", "true", "false", digit, alpha, "/", "*", "=", "-", "+", "!", "%", "<", ">", "&", "|", "^"
    enum class lemonscript_expressions::TK : char { // Note, the characters are only for debugging purposes
        LPAREN = '(',
        COMMA = ',',
        RPAREN = ')',
        BOOLEAN_TRUE = 'T',
        BOOLEAN_FALSE = 'F',
        DIGIT = 'd',
        ALPHA = 'a',
        UNDERSCORE = '_',
        FORWARD_SLASH = '/',
        ASTERISK = '*',
        EQUALS = '=',
        MINUS = '-',
        PLUS = '+',
        EXCLAMATION_MARK = '!',
        PERCENT = '%',
        LANGLE = '<',
        RANGLE = '>',
        AMPERSAND = '&',
        PIPE = '|',
        CARET = '^',
        PERIOD = '.',
        END_OF_FILE = 'E'
    };
    
    struct lemonscript_expressions::Token {
        lemonscript_expressions::TK kind;
        std::string string;
        Token(lemonscript_expressions::TK k, const std::string &s) : kind(k), string(s) { };
        std::string description() { return "(" + std::to_string(static_cast<char>(kind)) + ", " + string + ")"; }
    };
    
    class lemonscript_expressions::ExpressionScanner {
        std::string toParse;
        size_t readIndex;
        
        bool got_eof; // true iff have seen EOF
        // (int rather than char to handle EOF)
        static const int EndOfFile = -1;
        
        int getchar();
        
    public:
        ExpressionScanner(const std::string &s) : toParse(s), readIndex(0), got_eof(false) { };
        Token scan();
    };

#endif /* ExpressionScanner_hpp */
