//
//  OperatorType.hpp
//  lemonscript
//
//  Created by Donald Pinckney on 2/28/16.
//  Copyright © 2016 Donald Pinckney. All rights reserved.
//

#ifndef OperatorType_hpp
#define OperatorType_hpp

#include <stdio.h>

#include <vector>
#include <map>
#include <functional>
#include <string>

#include "expressions.h"

#include "AvailableCppCommandDeclaration.h"

struct lemonscript_expressions::TypeSpecification {
    std::vector<lemonscript::DataType> inputTypes;
    lemonscript::DataType returnType;
    std::function<int (std::vector<int>)> func;
};

class lemonscript_expressions::OperatorType {
    
    static std::map<std::string, lemonscript_expressions::OperatorType> operatorTypeMemoization;
    
public:
    
    bool isIdentityOperator; // True iff (i)T = T and (i)x = x
    std::string operatorText;
    int precedence = -1; // Ranked 0 to 999, 999 being highest precedence
    std::vector<lemonscript_expressions::TypeSpecification> specifications;
    
    
    static OperatorType identity();
    
    static OperatorType lookupOperatorType(std::string opString);
};


#endif /* OperatorType_hpp */
