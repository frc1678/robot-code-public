//
//  ExpressionScanner.cpp
//  lemonscript
//
//  Created by Donald Pinckney on 2/22/16.
//  Copyright © 2016 Donald Pinckney. All rights reserved.
//

#include "ExpressionScanner.h"

#include <ctype.h>

using namespace lemonscript_expressions;

std::ostream & lemonscript_expressions::operator<<(std::ostream &os, const Token &tok) {
    os << "(" << static_cast<char>(tok.kind) << ", " << tok.string << ")";
    return os;
}


bool myisalpha(char c) {
    return isalpha(c);
}

bool myisdigit(char c) {
    return isdigit(c);
}

Token ccase1(char c, TK r) {
    return Token(r, std::string(1, c));
}
Token scase1(const std::string &s, TK r) {
    return Token(r, s);
}

// call to advance token stream.
// acts as a generator (iterator) over input.
// returns Token
Token ExpressionScanner::scan() {
    if (got_eof) {
        throw "scan: oops -- called after eof.";
    }
    
    while (true) {
        int c = getchar();
        
        if(c == 't') {
            size_t startIndex = readIndex;
            
            int cr = getchar();
            int cu = getchar();
            int ce = getchar();
            if(cr == 'r' && cu == 'u' && ce == 'e') {
                return Token(TK::BOOLEAN_TRUE, "true");
            }
            
            readIndex = startIndex;
        } else if(c == 'f') {
            size_t startIndex = readIndex;

            int ca = getchar();
            int cl = getchar();
            int cs = getchar();
            int ce = getchar();
            if(ca == 'a' && cl == 'l' && cs == 's' && ce == 'e') {
                return Token(TK::BOOLEAN_FALSE, "false");
            }
            
            readIndex = startIndex;
        }
        
        if (myisalpha((char)c)) {
            return ccase1(c, TK::ALPHA);
        } else if (myisdigit((char) c)) {
            return ccase1(c, TK::DIGIT);
        } else {
// "(", ",", ")", "true", "false", digit, alpha, "/", "*", "=", "-", "+", "!", "%", "<", ">", "&", "|", "^"
            switch (c) {
                case '_':
                    return ccase1(c, TK::UNDERSCORE);
                case '(':
                    return ccase1(c, TK::LPAREN);
                case ',':
                    return ccase1(c, TK::COMMA);
                case ')':
                    return ccase1(c, TK::RPAREN);
                case '/':
                    return ccase1(c, TK::FORWARD_SLASH);
                case '*':
                    return ccase1(c, TK::ASTERISK);
                case '=':
                    return ccase1(c, TK::EQUALS);
                case '-':
                    return ccase1(c, TK::MINUS);
                case '+':
                    return ccase1(c, TK::PLUS);
                case '!':
                    return ccase1(c, TK::EXCLAMATION_MARK);
                case '%':
                    return ccase1(c, TK::PERCENT);
                case '<':
                    return ccase1(c, TK::LANGLE);
                case '>':
                    return ccase1(c, TK::RANGLE);
                case '&':
                    return ccase1(c, TK::AMPERSAND);
                case '|':
                    return ccase1(c, TK::PIPE);
                case '^':
                    return ccase1(c, TK::CARET);
                case '.':
                    return ccase1(c, TK::PERIOD);
                case EndOfFile:
                    got_eof = true;
                    return Token(TK::END_OF_FILE, "*EOF*");
                                        
                case ' ':
                case '\t':
                case '\r': // for Windows (lines end in rn)
                    break; // whitespace is easy to ignore
                    
                default:
                    throw "scan: bad char (ASCII " + std::to_string(c) + ")";
                    break;
            }
        }
    }
}


int ExpressionScanner::getchar() {
    if(readIndex >= toParse.length()) {
        return -1;
    }
    char c = toParse[readIndex];
    readIndex++;
    
    return c;
}





