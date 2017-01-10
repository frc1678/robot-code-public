//
//  PrefixExpression.cpp
//  lemonscript
//
//  Created by Donald Pinckney on 2/28/16.
//  Copyright © 2016 Donald Pinckney. All rights reserved.
//

#include "PrefixExpression.h"



std::ostream & lemonscript_expressions::operator<<(std::ostream &o, lemonscript_expressions::PrefixExpression pre) {
    o << pre.prefixOperator;
    
    if (pre.isAtom) {
        o << pre.atom;
    } else {
        o << "(";
        for (auto it = pre.expressionList.begin(); it != pre.expressionList.end(); ++it) {
            if(it != pre.expressionList.begin()) {
                o << " ";
            }
            o << *it;
        }
        o << ")";
    }
    
    return o;
}