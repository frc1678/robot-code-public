//
//  Expression.hpp
//  lemonscript
//
//  Created by Donald Pinckney on 2/6/16.
//  Copyright © 2016 Donald Pinckney. All rights reserved.
//

#ifndef Expression_hpp
#define Expression_hpp

#include <stdio.h>
#include <string>

#include "lemonscript.h"
#include "expressions.h"

#include "AvailableCppCommandDeclaration.h"
#include "LemonScriptState.h"
#include "ExpressionTree.h"

class lemonscript_expressions::Expression {
    
    bool isConst;
    bool didSupertype;
    lemonscript::DataType t;
    int constValue;
    
    ExpressionTree *tree;
    
public:
    Expression(const std::string &str, lemonscript::LemonScriptState *state, int lineNum);
    Expression(const std::string &str, lemonscript::DataType forceType, lemonscript::LemonScriptState *state, int lineNum);

    ~Expression();
    
    bool neededToSupertype() const;
    bool isConstant() const;
    lemonscript::DataType getType() const;
    void getValue(void *p) const;
};


#endif /* Expression_hpp */
