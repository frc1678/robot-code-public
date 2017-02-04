//
//  expressions.h
//  lemonscript
//
//  Created by Donald Pinckney on 2/22/16.
//  Copyright © 2016 Donald Pinckney. All rights reserved.
//

#ifndef expressions_h
#define expressions_h

#include <iostream>
#include <string>

namespace lemonscript_expressions {
    class Expression;
    class ExpressionScanner;
    class ExpressionParser;
    class ExpressionListItem;
    class OperatorType;
    struct TypeSpecification;
    class PrefixExpression;
    class Atom;
    class ExpressionTree;
    struct Token;
    struct ExpressionTreeRecurseAttributes;
    enum class TK : char;
    enum class NonTerminal;
    
    struct HaltError {
        std::string message;
    };
    
    std::ostream & operator<<(std::ostream &os, const Token &tok);
    std::ostream & operator<<(std::ostream &o, const lemonscript_expressions::Atom &a);
    std::ostream & operator<<(std::ostream &o, lemonscript_expressions::PrefixExpression pre);
    std::ostream & operator<<(std::ostream &o, lemonscript_expressions::OperatorType opType);
    std::ostream & operator<<(std::ostream &o, lemonscript_expressions::ExpressionListItem pre);

}

#endif /* expressions_h */
