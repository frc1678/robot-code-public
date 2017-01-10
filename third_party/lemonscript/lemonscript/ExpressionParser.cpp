//
//  ExpressionParser.cpp
//  lemonscript
//
//  Created by Donald Pinckney on 2/22/16.
//  Copyright © 2016 Donald Pinckney. All rights reserved.
//

#include "ExpressionParser.h"

#include <algorithm>
#include <iostream>

using namespace lemonscript_expressions;
using namespace lemonscript;
using namespace std;

#define parse_error(s) throw (s)

ExpressionParser::ExpressionParser(const string &toParse) : scanner(toParse), tok(scanner.scan()) {
    prefixExpression = expression();
    
    if(tok.kind != TK::END_OF_FILE) {
        parse_error("Junk after end of expression");
    }
}


//  expression = prefix_expression [binary_expressions];
PrefixExpression ExpressionParser::expression() {
    PrefixExpression pre = prefix_expression();
    
    ExpressionListItem preItem;
    preItem.isOperator = false;
    preItem.setPrefixExpression(pre);
    
    vector<ExpressionListItem> binaryExps;
    if(inFirstSet(NonTerminal::binary_expressions)) {
        binaryExps = binary_expressions();
    }
    
    PrefixExpression ret;
    ret.prefixOperator = OperatorType::identity();
    ret.expressionList = {preItem};
    ret.expressionList.insert(ret.expressionList.end(), binaryExps.begin(), binaryExps.end());
    
    return ret;
}

// //  parenthesized_expression = "(" expression {"," expression} ")";
//  parenthesized_expression = "(" expression ")";
vector<ExpressionListItem> ExpressionParser::parenthesized_expression() {
    mustbe(TK::LPAREN);
    
    PrefixExpression pre = expression();
    
    // Disabled comma separated expressions for now
//    while (is(TK::COMMA)) {
//        mustbe(TK::COMMA);
//        expression();
//    }
    
    mustbe(TK::RPAREN);
    
    return pre.expressionList;
}


//  prefix_expression = [prefix_operator] postfix_expression;
PrefixExpression ExpressionParser::prefix_expression() {
    OperatorType op = OperatorType::identity();
    
    if(inFirstSet(NonTerminal::prefix_operator)) {
        op = prefix_operator();
    }
    
    PrefixExpression ret;
    ret.expressionList = postfix_expression();
    ret.prefixOperator = op;
    
    return ret;
}

//  binary_expression = binary_operator prefix_expression;
vector<ExpressionListItem> ExpressionParser::binary_expression() {
    OperatorType op = binary_operator();
    PrefixExpression pre = prefix_expression();
    
    ExpressionListItem opItem;
    opItem.isOperator = true;
    opItem.setOperatorType(op);
    
    ExpressionListItem preItem;
    preItem.isOperator = false;
    preItem.setPrefixExpression(pre);
    
    return {opItem, preItem};
}

//  binary_expressions = binary_expression [binary_expressions];
vector<ExpressionListItem> ExpressionParser::binary_expressions() {
    vector<ExpressionListItem> items;
    
    vector<ExpressionListItem> tempItems = binary_expression();
    items.insert(items.end(), tempItems.begin(), tempItems.end());
    
    
    if(inFirstSet(NonTerminal::binary_expressions)) {
        tempItems = binary_expressions();
        
        items.insert(items.end(), tempItems.begin(), tempItems.end());
    }
    
    return items;
}


//  postfix_expression = primary_expression
vector<ExpressionListItem> ExpressionParser::postfix_expression() {
    return primary_expression();
}

//  primary_expression = identifier | literal | parenthesized_expression;
vector<ExpressionListItem> ExpressionParser::primary_expression() {
    if(inFirstSet(NonTerminal::identifier)) {
        Atom a = identifier();
        PrefixExpression pre;
        pre.isAtom = true;
        pre.atom = a;
        
        ExpressionListItem preItem;
        preItem.isOperator = false;
        preItem.setPrefixExpression(pre);
        
        return {preItem};
    } else if(inFirstSet(NonTerminal::literal)) {
        Atom a = literal();
        
        PrefixExpression pre;
        pre.isAtom = true;
        pre.atom = a;
        
        ExpressionListItem preItem;
        preItem.isOperator = false;
        preItem.setPrefixExpression(pre);
        
        return {preItem};
    } else if(inFirstSet(NonTerminal::parenthesized_expression)) {
        return parenthesized_expression();
    } else {
        parse_error("Missing token");
    }
}

//  prefix_operator = operator;
OperatorType ExpressionParser::prefix_operator() {
    OperatorType op = _operator();
    
    // Filter to unary operators only
    vector<TypeSpecification> specs(op.specifications.size());
    auto it = std::copy_if(op.specifications.begin(), op.specifications.end(), specs.begin(), [](TypeSpecification s) {
        return s.inputTypes.size() == 1;
    });
    specs.resize(std::distance(specs.begin(), it));
    
    op.specifications = specs;
    
    return op;
}

//  binary_operator = operator;
OperatorType ExpressionParser::binary_operator() {
    OperatorType op = _operator();
    
    // Filter to binary operators only
    vector<TypeSpecification> specs(op.specifications.size());
    auto it = std::copy_if(op.specifications.begin(), op.specifications.end(), specs.begin(), [](TypeSpecification s) {
        return s.inputTypes.size() == 2;
    });
    specs.resize(std::distance(specs.begin(), it));
    
    op.specifications = specs;
    
    return op;
}

//  operator = operator_character {operator_character};
OperatorType ExpressionParser::_operator() {
    string opString = operator_character();
    while (inFirstSet(NonTerminal::operator_character) && inFirstSet(NonTerminal::prefix_operator) == false) {
        opString += operator_character();
    }
    
    OperatorType opType = OperatorType::lookupOperatorType(opString);
    
    return opType;
}

//  operator_character = "/" | "*" | "=" | "-" | "+" | "!" | "%" | "<" | ">" | "&" | "|" | "^";
std::string ExpressionParser::operator_character() {
    if (is(TK::FORWARD_SLASH)) {
        return mustbe(TK::FORWARD_SLASH);
    } else if (is(TK::ASTERISK)) {
        return mustbe(TK::ASTERISK);
    } else if (is(TK::EQUALS)) {
        return mustbe(TK::EQUALS);
    } else if (is(TK::MINUS)) {
        return mustbe(TK::MINUS);
    } else if (is(TK::PLUS)) {
        return mustbe(TK::PLUS);
    } else if (is(TK::EXCLAMATION_MARK)) {
        return mustbe(TK::EXCLAMATION_MARK);
    } else if (is(TK::PERCENT)) {
        return mustbe(TK::PERCENT);
    } else if (is(TK::LANGLE)) {
        return mustbe(TK::LANGLE);
    } else if (is(TK::RANGLE)) {
        return mustbe(TK::RANGLE);
    } else if (is(TK::AMPERSAND)) {
        return mustbe(TK::AMPERSAND);
    } else if (is(TK::PIPE)) {
        return mustbe(TK::PIPE);
    } else if (is(TK::CARET)) {
        return mustbe(TK::CARET);
    } else {
        parse_error("Missing token");
    }
}

//  identifier = alpha {identifier_character};
Atom ExpressionParser::identifier() {
    string id = alpha();
    
    while (inFirstSet(NonTerminal::identifier_character)) {
        id += identifier_character();
    }
    
    Atom a;
    a.isIdentifier = true;
    a.text = id;
    
    return a;
}

//  identifier_character = alpha | digit;
std::string ExpressionParser::identifier_character() {
    if(inFirstSet(NonTerminal::alpha)) {
        return alpha();
    } else if(inFirstSet(NonTerminal::digit)) {
        return digit();
    } else {
        parse_error("Missing token");
    }
}

//  literal = numeric_literal | boolean_literal;
Atom ExpressionParser::literal() {
    if(inFirstSet(NonTerminal::numeric_literal)) {
        return numeric_literal();
    } else if(inFirstSet(NonTerminal::boolean_literal)) {
        return boolean_literal();
    } else {
        parse_error("Missing token");
    }
}

//  boolean_literal = "true" | "false";
Atom ExpressionParser::boolean_literal() {
    string text;
    if(is(TK::BOOLEAN_TRUE)) {
        text = mustbe(TK::BOOLEAN_TRUE);
    } else if(is(TK::BOOLEAN_FALSE)) {
        text = mustbe(TK::BOOLEAN_FALSE);
    } else {
        parse_error("Missing token");
    }
    
    Atom a;
    a.isIdentifier = false;
    a.text = text;
    a.parsedType = DataType::BOOLEAN;
    
    return a;
}

//  numeric_literal = ["-"] integer_literal | ["-"] floating_point_literal;
Atom ExpressionParser::numeric_literal() {
    
    string negative = "";
    
    if(is(TK::MINUS)) {
        negative = mustbe(TK::MINUS);
    }
    
    string decimalString = decimal_literal();
    Atom a;
    a.isIdentifier = false;
    if(inFirstSet(NonTerminal::decimal_fraction)) {
        a.text = decimalString + decimal_fraction();
        a.parsedType = DataType::FLOAT;
    } else {
        a.text = decimalString;
        a.parsedType = DataType::INT;
    }
    
    return a;
}


//  decimal_fraction = "." decimal_literal;
std::string ExpressionParser::decimal_fraction() {
    return mustbe(TK::PERIOD) + decimal_literal();
}

//  decimal_literal = digit {digit};
std::string ExpressionParser::decimal_literal() {
    string text = digit();
    while (inFirstSet(NonTerminal::digit)) {
        text += digit();
    }
    
    return text;
}

//  alpha = "a".."z" | "A".."Z";
std::string ExpressionParser::alpha() {
    return mustbe(TK::ALPHA);
}

//  digit = "0".."9";
std::string ExpressionParser::digit() {
    return mustbe(TK::DIGIT);
}






bool ExpressionParser::is(lemonscript_expressions::TK tk) {
    return tok.kind == tk;
}

bool ExpressionParser::inFirstSet(lemonscript_expressions::NonTerminal nt) {
    set<TK> fs = firstSet(nt);
    return fs.find(tok.kind) != fs.end();
}

template <typename T>
set<T> sunion(set<T> x, const set<T> &y) {
    for (auto it = y.begin(); it != y.end(); ++it) {
        x.insert(*it);
    }
    return x;
}

set<TK> ExpressionParser::firstSet(lemonscript_expressions::NonTerminal nt) {
    switch (nt) {
        case NonTerminal::expression:
            return firstSet(NonTerminal::prefix_expression);
        case NonTerminal::parenthesized_expression:
            return {TK::LPAREN};
        case NonTerminal::prefix_expression:
            return firstSet(NonTerminal::prefix_operator);
        case NonTerminal::binary_expression:
            return firstSet(NonTerminal::binary_operator);
        case NonTerminal::binary_expressions:
            return firstSet(NonTerminal::binary_expression);
        case NonTerminal::postfix_expression:
            return firstSet(NonTerminal::primary_expression);
        case NonTerminal::primary_expression:
            return sunion(sunion(firstSet(NonTerminal::identifier), firstSet(NonTerminal::literal)), firstSet(NonTerminal::parenthesized_expression));
        case NonTerminal::prefix_operator:
            return {TK::MINUS, TK::EXCLAMATION_MARK};
        case NonTerminal::binary_operator:
            return firstSet(NonTerminal::_operator);
        case NonTerminal::postfix_operator:
            return firstSet(NonTerminal::_operator);
        case NonTerminal::_operator:
            return firstSet(NonTerminal::operator_character);
            
        case NonTerminal::operator_character:
            return {TK::FORWARD_SLASH, TK::ASTERISK, TK::EQUALS, TK::MINUS, TK::PLUS, TK::EXCLAMATION_MARK, TK::PERCENT, TK::LANGLE, TK::RANGLE, TK::AMPERSAND, TK::PIPE, TK::CARET};
            
        case NonTerminal::identifier:
            return firstSet(NonTerminal::alpha);
        case NonTerminal::identifier_character:
            return sunion(firstSet(NonTerminal::alpha), firstSet(NonTerminal::digit));
        case NonTerminal::literal:
            return sunion(firstSet(NonTerminal::numeric_literal), firstSet(NonTerminal::boolean_literal));
        case NonTerminal::boolean_literal:
            return {TK::BOOLEAN_TRUE, TK::BOOLEAN_FALSE};
        case NonTerminal::numeric_literal:
            return sunion({TK::MINUS}, firstSet(NonTerminal::decimal_literal));
        case NonTerminal::decimal_fraction:
            return {TK::PERIOD};
        case NonTerminal::decimal_literal:
            return firstSet(NonTerminal::digit);
        case NonTerminal::alpha:
            return {TK::ALPHA};
        case NonTerminal::digit:
            return {TK::DIGIT};
    }
    
    // This suppresses warnings with Travis CI compiler that isn't good enough to prove return type above.
    cerr << "Error: Unknown Non-terminal: " << (int)nt << endl;
    return {};
}

void ExpressionParser::scan() {
    tok = scanner.scan();
}

string ExpressionParser::mustbe(TK tk) {
    if( tok.kind != tk ) {
        parse_error("mustbe: want " + to_string(static_cast<char>(tk)) + ", got " + tok.description());
    }
    string temp = tok.string;
    scan();
    
    return temp;
}

