//
//  Expression.cpp
//  lemonscript
//
//  Created by Donald Pinckney on 2/6/16.
//  Copyright © 2016 Donald Pinckney. All rights reserved.
//

#include "Expression.h"

#include "ExpressionParser.h"

using namespace lemonscript_expressions;
using lemonscript::DataType;
using namespace std;


Expression::Expression(const std::string &str, lemonscript::LemonScriptState *state, int lineNum) {
    
    ExpressionParser scan(str);

    tree = new ExpressionTree(scan.getRootPrefixExpression(), state);
    ExpressionTreeRecurseAttributes attribs = tree->compileTree();
    if (attribs.CANT_MATCH_TYPE) {
        throw "Can't resolve type tree, line: " + to_string(lineNum);
    }
    
    
    t = attribs.dataType;
    isConst = attribs.isConstant;
    didSupertype = attribs.neededToSupertype;
    
    if(attribs.isConstant) {
        size_t dataSize;
        if (t == lemonscript::DataType::INT) {
            dataSize = sizeof(int);
        } else if(t == lemonscript::DataType::FLOAT) {
            dataSize = sizeof(float);
        } else if(t == lemonscript::DataType::BOOLEAN) {
            dataSize = sizeof(bool);
        } else {
            throw "Type expressions not yet supported";
        }
        
        
        int val = tree->evaluate();
        bzero(&constValue, sizeof(int));
        memcpy(&constValue, &val, dataSize);
    }
}

Expression:: Expression(const std::string &str, lemonscript::DataType forceType, lemonscript::LemonScriptState *state, int lineNum) {
    ExpressionParser scan(str);
    
    tree = new ExpressionTree(scan.getRootPrefixExpression(), state);
    ExpressionTreeRecurseAttributes attribs = tree->compileTree(forceType);
    if (attribs.CANT_MATCH_TYPE) {
        throw "Can't resolve type tree, line: " + to_string(lineNum);
    }
    
    
    t = attribs.dataType;
    isConst = attribs.isConstant;
    didSupertype = attribs.neededToSupertype;
    
    if(attribs.isConstant) {
        size_t dataSize;
        if (t == lemonscript::DataType::INT) {
            dataSize = sizeof(int);
        } else if(t == lemonscript::DataType::FLOAT) {
            dataSize = sizeof(float);
        } else if(t == lemonscript::DataType::BOOLEAN) {
            dataSize = sizeof(bool);
        } else {
            throw "Type expressions not yet supported";
        }
        
        
        int val = tree->evaluate();
        bzero(&constValue, sizeof(int));
        memcpy(&constValue, &val, dataSize);
    }
}


Expression::~Expression() {
    delete tree;
}

bool Expression::neededToSupertype() const {
    return didSupertype;
}

bool Expression::isConstant() const {
    return isConst;
}

DataType Expression::getType() const {
    return t;
}

void Expression::getValue(void *p) const {
    size_t dataSize;
    if (t == lemonscript::DataType::INT) {
        dataSize = sizeof(int);
    } else if(t == lemonscript::DataType::FLOAT) {
        dataSize = sizeof(float);
    } else if(t == lemonscript::DataType::BOOLEAN) {
        dataSize = sizeof(bool);
    } else {
        throw "Type expressions not yet supported";
    }
    
    if(isConst) {
        memcpy(p, &constValue, dataSize);
        return;
    }
    
    int val = tree->evaluate();
    
    memcpy(p, &val, dataSize);
}