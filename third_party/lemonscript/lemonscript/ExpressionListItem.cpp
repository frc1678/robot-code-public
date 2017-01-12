//
//  ExpressionListItem.cpp
//  lemonscript
//
//  Created by Donald Pinckney on 2/28/16.
//  Copyright © 2016 Donald Pinckney. All rights reserved.
//

#include "ExpressionListItem.h"

using namespace lemonscript_expressions;

void ExpressionListItem::setOperatorType(OperatorType t) {
//    if(operatorType != NULL) {
//        delete operatorType;
//    }
    operatorType = new lemonscript_expressions::OperatorType(t);
}

void ExpressionListItem::setPrefixExpression(PrefixExpression pre) {
//    if(prefixExpression != NULL) {
//        delete prefixExpression;
//    }
    prefixExpression = new lemonscript_expressions::PrefixExpression(pre);
}

std::ostream & lemonscript_expressions::operator<<(std::ostream &o, lemonscript_expressions::ExpressionListItem item) {
    if(item.isOperator) {
        o << *(item.operatorType);
    } else {
        o << *(item.prefixExpression);
    }
    
    return o;
}

//ExpressionListItem::~ExpressionListItem() {
//    if(operatorType != NULL) {
//        delete operatorType;
//    }
//    if(prefixExpression != NULL) {
//        delete prefixExpression;
//    }
//}