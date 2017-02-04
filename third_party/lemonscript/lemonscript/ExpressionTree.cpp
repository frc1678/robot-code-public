//
//  ExpressionTree.cpp
//  lemonscript
//
//  Created by Donald Pinckney on 2/29/16.
//  Copyright © 2016 Donald Pinckney. All rights reserved.
//

#include "ExpressionTree.h"
#include "Atom.h"
#include "LemonScriptState.h"

#include <algorithm>

using namespace lemonscript_expressions;
using namespace lemonscript;
using namespace std;

ExpressionTree::ExpressionTree(PrefixExpression rootExp, LemonScriptState *s) {
    
    state = s;
    
    // Using rules of precedence, build the tree
    
    // Cases:
    // 0. Prefix expression is atom: terminate recursion
    // 1. Prefix operator is identity: skip identity in tree building, and continue to 3.
    // 2. Prefix operator is not identity: build prefix operator into tree, and recurse on expressionList
    // 3. If expressionList is empty, throw syntax error
    // 4. Otherwise, find lowest precedence to the rightmost operator, split expressionList left and right of it, and recurse.
    
    this->leftTree = NULL;
    this->rightTree = NULL;
    
    if(rootExp.isAtom) {
        this->atom = rootExp.atom;
    } else if(rootExp.prefixOperator.isIdentityOperator == false) {
        this->op = rootExp.prefixOperator;
        
        PrefixExpression toRecurse = rootExp;
        toRecurse.prefixOperator = OperatorType::identity();
        
        this->leftTree = new ExpressionTree(toRecurse, s);
    } else if(rootExp.prefixOperator.isIdentityOperator == true && rootExp.expressionList.size() == 1) {
        ExpressionTree *actualTree = new ExpressionTree(*(rootExp.expressionList[0].prefixExpression), s);
        this->leftTree = actualTree->leftTree;
        this->rightTree = actualTree->rightTree;
        
        // Transfer ownership to our own tree, so these don't get deleted
        actualTree->leftTree = NULL;
        actualTree->rightTree = NULL;
        
        this->atom = actualTree->atom;
        this->op = actualTree->op;
        delete actualTree;
    } else {
        // Now we search for the lowest precedence, rightmost operator
        int lowestPrecedence = 1000;
        
        vector<ExpressionListItem>::iterator lowestPrecedenceIt = rootExp.expressionList.end();
        
        for (auto it = rootExp.expressionList.begin(); it != rootExp.expressionList.end(); ++it) {
            if(it->isOperator == false) {
                continue;
            }
            
            OperatorType *opType = it->operatorType;
            if(opType->precedence <= lowestPrecedence) {
                lowestPrecedence = opType->precedence;
                lowestPrecedenceIt = it;
            }
        }
        
        if (lowestPrecedenceIt == rootExp.expressionList.end()) {
            throw "Missing operator";
        }
        
        
        auto index = lowestPrecedenceIt - rootExp.expressionList.begin();
        vector<ExpressionListItem> leftExpressionList(index);
        vector<ExpressionListItem> rightExpressionList(rootExp.expressionList.size() - index - 1);
        
        copy(rootExp.expressionList.begin(), lowestPrecedenceIt, leftExpressionList.begin());
        copy(lowestPrecedenceIt + 1, rootExp.expressionList.end(), rightExpressionList.begin());
        
        PrefixExpression leftPrefixExpression;
        PrefixExpression rightPrefixExpression;
        
        if(leftExpressionList.size() == 1) {
            ExpressionListItem item = leftExpressionList[0];
            if(item.isOperator) {
                throw "Extra operator";
            }
            
            leftPrefixExpression = *(leftExpressionList[0].prefixExpression);
        } else {
            leftPrefixExpression.prefixOperator = OperatorType::identity();
            leftPrefixExpression.isAtom = false;
            leftPrefixExpression.expressionList = leftExpressionList;
        }
        
        if(rightExpressionList.size() == 1) {
            ExpressionListItem item = rightExpressionList[0];
            if(item.isOperator) {
                throw "Extra operator";
            }
            
            rightPrefixExpression = *(rightExpressionList[0].prefixExpression);
        } else {
            rightPrefixExpression.prefixOperator = OperatorType::identity();
            rightPrefixExpression.isAtom = false;
            rightPrefixExpression.expressionList = rightExpressionList;
        }
        
        
        this->op = *lowestPrecedenceIt->operatorType;
        this->leftTree = new ExpressionTree(leftPrefixExpression, s);
        this->rightTree = new ExpressionTree(rightPrefixExpression, s);
    }
}

ExpressionTreeRecurseAttributes ExpressionTree::compileTree() {
    vector<DataType> toTry = {DataType::FLOAT, DataType::INT, DataType::BOOLEAN};
    
    int matchCount = 0;
    ExpressionTreeRecurseAttributes matchedRecurse;
    
    for (auto it = toTry.begin(); it != toTry.end(); ++it) {
        ExpressionTreeRecurseAttributes recurse = compileTree(*it);
        if(recurse.CANT_MATCH_TYPE) {
            continue;
        }
        
        matchCount++;
        matchedRecurse = recurse;
        
        if(!recurse.neededToSupertype) {
            matchCount = 1;
            break;
        }
    }
    
    if(matchCount == 0) {
        throw "Could not find override for operator: " + op.operatorText;
    } else if(matchCount >= 2) {
        throw "Can not disambiguate overrides for operator: " + op.operatorText;
    }
    
    return matchedRecurse;
}

ExpressionTreeRecurseAttributes ExpressionTree::compileTree(DataType neededType) {
    if(leftTree == NULL && rightTree == NULL) {
        DataType currentType;
        ExpressionTreeRecurseAttributes recurseValues;

        if(atom.isIdentifier) {
            if(state->addressOfVariable(atom.text) == NULL) {
                HaltError err;
                err.message = "Unknown variable: " + atom.text;
                throw err;
            }
            currentType = state->typeOfVariable(atom.text);
            recurseValues.isConstant = false;
        } else {
            currentType = atom.parsedType;
            recurseValues.isConstant = true;
        }
        
        
        if(DataTypeIsSubtypeOf(currentType, neededType)) {
            
            if(atom.isIdentifier) {
                void *address = state->addressOfVariable(atom.text);
                int copySize = 0;
                if(currentType == DataType::BOOLEAN) {
                    copySize = sizeof(bool);
                } else if(currentType == DataType::FLOAT) {
                    copySize = sizeof(float);
                } else if(currentType == DataType::INT) {
                    copySize = sizeof(int);
                }
                
                func = [address, copySize] (vector<int> args) {
                    int retVal;
                    bzero(&retVal, sizeof(int));
                    memcpy(&retVal, address, copySize);
                    return retVal;
                };
            } else {
                
                int parsedValue;
                bzero(&parsedValue, sizeof(int));
                if(currentType == DataType::BOOLEAN) {
                    bool boolVal = (atom.text == "true" ? true : false);
                    
                    parsedValue = DataTypeBuildInt(boolVal);
                } else if(currentType == DataType::INT) {
                    int intVal = stoi(atom.text);
                    
                    parsedValue = DataTypeBuildInt(intVal);
                } else if(currentType == DataType::FLOAT) {
                    float floatVal = stof(atom.text);
                    
                    parsedValue = DataTypeBuildInt(floatVal);
                }
                
                parsedValue = DataTypeIntCastFromTo(currentType, neededType, parsedValue);
                
                
                func = [parsedValue] (vector<int> args) {
                    return parsedValue;
                };
            }
            
            recurseValues.neededToSupertype = neededType != currentType;
            recurseValues.dataType = neededType;
            recurseValues.CANT_MATCH_TYPE = false;
            
            return recurseValues;
        } else {
            recurseValues.CANT_MATCH_TYPE = true;
            recurseValues.neededToSupertype = true;
            
            return recurseValues;
        }
    } else {
        
        vector<ExpressionTree *> treeArgs;
        treeArgs.push_back(leftTree);
        if(rightTree) {
            treeArgs.push_back(rightTree);
        }
        
        
        
        vector<ExpressionTreeRecurseAttributes> matchedChildRecurseResults;
        TypeSpecification matchedTypeSpec;
        bool areAllChildrenConstant = true;
        
        int matchCount = 0;
        for (auto it = op.specifications.begin(); it != op.specifications.end(); ++it) {
            TypeSpecification spec = *it;
            if (spec.inputTypes.size() != treeArgs.size()) {
                continue;
            }
            
            bool perfectMatch = true;
            bool match = true;
            
            vector<ExpressionTreeRecurseAttributes> childRecurseResults;
            for (size_t childIndex = 0; childIndex < treeArgs.size(); childIndex++) {
                ExpressionTreeRecurseAttributes childRecurse = treeArgs[childIndex]->compileTree(spec.inputTypes[childIndex]);
                childRecurseResults.push_back(childRecurse);
                
                if(childRecurse.isConstant == false) {
                    areAllChildrenConstant = false;
                }
                
                if(childRecurse.CANT_MATCH_TYPE) {
                    match = false;
                    break;
                }
                
                if(childRecurse.neededToSupertype) {
                    perfectMatch = false;
                }
            }
            
            if(!match) {
                continue;
            }
            
            matchedChildRecurseResults = childRecurseResults;
            matchedTypeSpec = spec;
            matchCount++;

            if(perfectMatch) { // Unambigous for certain
                matchCount = 1;
                break;
            }
        }
        
        if(matchCount == 0) {
            throw "Could not find override for operator: " + op.operatorText;
        } else if(matchCount >= 2) {
            throw "Can not disambiguate overrides for operator: " + op.operatorText;
        }
        
        
        ExpressionTreeRecurseAttributes recursionResult;
        DataType actualReturnType = matchedTypeSpec.returnType;
        if (actualReturnType == neededType) {
            recursionResult.CANT_MATCH_TYPE = false;
            recursionResult.dataType = neededType;
            recursionResult.isConstant = areAllChildrenConstant;
            recursionResult.neededToSupertype = false;
            func = matchedTypeSpec.func;
        } else if(DataTypeIsSubtypeOf(actualReturnType, neededType)) {
            recursionResult.CANT_MATCH_TYPE = false;
            recursionResult.dataType = neededType;
            recursionResult.isConstant = areAllChildrenConstant;
            recursionResult.neededToSupertype = true;
            
            if(neededType == DataType::BOOLEAN && actualReturnType == DataType::FLOAT) {
                func = [matchedTypeSpec] (vector<int> args) {
                    int retVal = matchedTypeSpec.func(args);
                    return DataTypeIntCastFromTo(DataType::FLOAT, DataType::BOOLEAN, retVal);
                };
            }
            if(neededType == DataType::BOOLEAN && actualReturnType == DataType::INT) {
                func = [matchedTypeSpec] (vector<int> args) {
                    int retVal = matchedTypeSpec.func(args);
                    return DataTypeIntCastFromTo(DataType::INT, DataType::BOOLEAN, retVal);
                };
            }
            if(neededType == DataType::FLOAT && actualReturnType == DataType::BOOLEAN) {
                func = [matchedTypeSpec] (vector<int> args) {
                    int retVal = matchedTypeSpec.func(args);
                    return DataTypeIntCastFromTo(DataType::BOOLEAN, DataType::FLOAT, retVal);
                };
            }
            if(neededType == DataType::FLOAT && actualReturnType == DataType::INT) {
                func = [matchedTypeSpec] (vector<int> args) {
                    int retVal = matchedTypeSpec.func(args);
                    return DataTypeIntCastFromTo(DataType::INT, DataType::FLOAT, retVal);
                };
            }
            if(neededType == DataType::INT && actualReturnType == DataType::BOOLEAN) {
                func = [matchedTypeSpec] (vector<int> args) {
                    int retVal = matchedTypeSpec.func(args);
                    return DataTypeIntCastFromTo(DataType::BOOLEAN, DataType::INT, retVal);
                };
            }
            if(neededType == DataType::INT && actualReturnType == DataType::FLOAT) {
                func = [matchedTypeSpec] (vector<int> args) {
                    int retVal = matchedTypeSpec.func(args);
                    return DataTypeIntCastFromTo(DataType::FLOAT, DataType::INT, retVal);
                };
            }
        } else {
            recursionResult.CANT_MATCH_TYPE = true;
        }
        
        return recursionResult;
    }
}



int ExpressionTree::evaluate() {
    if (leftTree == NULL && rightTree == NULL) {
        return func({});
    } else if(rightTree == NULL) {
        return func({leftTree->evaluate()});
    } else if(leftTree != NULL && rightTree != NULL) {
        return func({leftTree->evaluate(), rightTree->evaluate()});
    } else {
        std::cerr << "This should never have happened" << std::endl;
        exit(1);
        return 0;
    }
}




void ExpressionTree::print(std::ostream &os, int depth) const {

    
    if(leftTree == NULL && rightTree == NULL) { // atom
        for (int i = 0; i < depth; i++) {
            os << "  ";
        }
        
        os << atom;
    } else if(rightTree == NULL) { // prefix operator
        for (int i = 0; i < depth; i++) {
            os << "  ";
        }
        
        os << op << endl;
        leftTree->print(os, depth + 1);
    } else if(leftTree != NULL && rightTree != NULL) {
        leftTree->print(os, depth + 1);
        os << endl;
        
        for (int i = 0; i < depth; i++) {
            os << "  ";
        }
        
        os << op << endl;
        
        rightTree->print(os, depth + 1);
    } else {
        std::cerr << "This should never have happened" << std::endl;
        exit(1);
    }
    
    if (depth == 0) {
        os << endl;
    }
}


ExpressionTree::~ExpressionTree() {
    if(leftTree) {
        delete leftTree;
    }
    if(rightTree) {
        delete rightTree;
    }
}

