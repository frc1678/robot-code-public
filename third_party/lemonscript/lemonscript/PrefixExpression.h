//
//  PrefixExpression.hpp
//  lemonscript
//
//  Created by Donald Pinckney on 2/28/16.
//  Copyright © 2016 Donald Pinckney. All rights reserved.
//

#ifndef PrefixExpression_hpp
#define PrefixExpression_hpp

#include <stdio.h>
#include <vector>
#include <string>
#include <iostream>

#include "expressions.h"
#include "OperatorType.h"
#include "ExpressionListItem.h"
#include "Atom.h"

class lemonscript_expressions::PrefixExpression {
    
public:
    
    lemonscript_expressions::OperatorType prefixOperator;
    
    bool isAtom = false;
    lemonscript_expressions::Atom atom;
    
    std::vector<lemonscript_expressions::ExpressionListItem> expressionList;
    
};


#endif /* PrefixExpression_hpp */
