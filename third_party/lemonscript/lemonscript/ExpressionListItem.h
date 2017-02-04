//
//  ExpressionListItem.hpp
//  lemonscript
//
//  Created by Donald Pinckney on 2/28/16.
//  Copyright © 2016 Donald Pinckney. All rights reserved.
//

#ifndef ExpressionListItem_hpp
#define ExpressionListItem_hpp

#include <stdio.h>

#include "expressions.h"

#include "OperatorType.h"
#include "PrefixExpression.h"

class lemonscript_expressions::ExpressionListItem {
    
    
public:
        
    bool isOperator; // Otherwise is prefix expression
    
    
    // Only defined if isOperator == true
    lemonscript_expressions::OperatorType *operatorType = nullptr;

    // Only defined if isOperator == false
    lemonscript_expressions::PrefixExpression *prefixExpression = nullptr;
    
    void setOperatorType(lemonscript_expressions::OperatorType t);
    
    void setPrefixExpression(lemonscript_expressions::PrefixExpression pre);
};


#endif /* ExpressionListItem_hpp */
