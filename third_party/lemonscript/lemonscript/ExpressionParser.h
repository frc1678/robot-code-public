//
//  ExpressionParser.hpp
//  lemonscript
//
//  Created by Donald Pinckney on 2/22/16.
//  Copyright © 2016 Donald Pinckney. All rights reserved.
//

#ifndef ExpressionParser_hpp
#define ExpressionParser_hpp

#include <stdio.h>
#include <string>
#include <set>
#include <vector>

#include "expressions.h"

#include "ExpressionScanner.h"

#include "ExpressionListItem.h"
#include "PrefixExpression.h"

enum class lemonscript_expressions::NonTerminal {
    expression,
    parenthesized_expression,
    prefix_expression,
    binary_expression,
    binary_expressions,
    postfix_expression,
    primary_expression,
    prefix_operator,
    binary_operator,
    postfix_operator,
    _operator,
    operator_character,
    identifier,
    identifier_character,
    literal,
    boolean_literal,
    numeric_literal,
    decimal_fraction,
    decimal_literal,
    alpha,
    digit
};

class lemonscript_expressions::ExpressionParser {
    ExpressionScanner scanner;
    Token tok;
    void scan();
    inline void parse_error(const std::string &s);
    std::string mustbe(TK tk);
    bool is(TK tk);
    bool inFirstSet(NonTerminal nt);
    std::set<TK> firstSet(NonTerminal nt);
    
    PrefixExpression expression();
    std::vector<ExpressionListItem> parenthesized_expression();
    
    PrefixExpression prefix_expression();
    
    std::vector<ExpressionListItem> binary_expression();
    std::vector<ExpressionListItem> binary_expressions();
    
    std::vector<ExpressionListItem> postfix_expression();
    
    std::vector<ExpressionListItem> primary_expression();
    
    OperatorType prefix_operator();
    OperatorType binary_operator();
    
    OperatorType _operator();
    std::string operator_character();
    
    Atom identifier();
    std::string identifier_character();
    
    Atom literal();
    Atom boolean_literal();
    Atom numeric_literal();
    
    std::string decimal_fraction();
    std::string decimal_literal();
    
    std::string alpha();
    std::string digit();
    
    
    PrefixExpression prefixExpression;
    
public:
    ExpressionParser(const std::string &toParse);
    
    PrefixExpression getRootPrefixExpression() { return prefixExpression; };
};

#endif /* ExpressionParser_hpp */
