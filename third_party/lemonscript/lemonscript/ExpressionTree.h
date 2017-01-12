//
//  ExpressionTree.hpp
//  lemonscript
//
//  Created by Donald Pinckney on 2/29/16.
//  Copyright © 2016 Donald Pinckney. All rights reserved.
//

#ifndef ExpressionTree_hpp
#define ExpressionTree_hpp

#include <stdio.h>
#include <iostream>
#include <functional>

#include "expressions.h"

#include "PrefixExpression.h"

std::ostream & operator<<(std::ostream &os, const lemonscript_expressions::ExpressionTree &tree);

struct lemonscript_expressions::ExpressionTreeRecurseAttributes {
    bool CANT_MATCH_TYPE; // !!!
    lemonscript::DataType dataType;
    bool isConstant;
    bool neededToSupertype;    
};

class lemonscript_expressions::ExpressionTree {
    
    lemonscript_expressions::ExpressionTree *leftTree = NULL;
    lemonscript_expressions::ExpressionTree *rightTree = NULL;
    
    
    std::function<int (std::vector<int>)> func;
    
    lemonscript_expressions::Atom atom;
    lemonscript_expressions::OperatorType op;
    
    void print(std::ostream &os, int depth) const;
    
    lemonscript::LemonScriptState *state;
    
public:
    ExpressionTree(PrefixExpression rootExp, lemonscript::LemonScriptState *s);
    ~ExpressionTree();
    
    lemonscript_expressions::ExpressionTreeRecurseAttributes compileTree(lemonscript::DataType neededType);
    lemonscript_expressions::ExpressionTreeRecurseAttributes compileTree();
    
    int evaluate();
    
    
    friend std::ostream & (::operator<<)(std::ostream &os, const lemonscript_expressions::ExpressionTree &tree);
    
};


#endif /* ExpressionTree_hpp */
