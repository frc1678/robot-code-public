//
//  OperatorType.cpp
//  lemonscript
//
//  Created by Donald Pinckney on 2/28/16.
//  Copyright © 2016 Donald Pinckney. All rights reserved.
//

#include "OperatorType.h"

#include <string.h>
#include <functional>
#include <cmath>

#include "AvailableCppCommandDeclaration.h"

using namespace lemonscript_expressions;
using namespace lemonscript;

using std::function;
using std::vector;
using std::map;
using std::string;

std::ostream & lemonscript_expressions::operator<<(std::ostream &o, OperatorType opType) {
    o << opType.operatorText;
    return o;
}

/*
 
const Operator<ExpressionDesc> DataType_unaryNegO("!.*", 1, true, operatorResolve(genericOperatorTypes({BOOLEAN}, {unaryNegO<bool>}, 1)));
const Operator<ExpressionDesc> DataType_unaryMathNegO("-.*", 1, true, operatorResolve(genericOperatorTypes({INT,FLOAT}, {unaryMathNegO<int>, unaryMathNegO<float>}, 1)));

const Operator<ExpressionDesc> DataType_expO(".*\\^.*", 2, true, operatorResolve(genericOperatorTypes({INT,FLOAT}, {binaryExpO<int>, binaryExpO<float>}, 2)));

const Operator<ExpressionDesc> DataType_modO(".*%.*", 2, true, operatorResolve(genericOperatorTypes({INT}, {binaryModO<int>}, 2)));
const Operator<ExpressionDesc> DataType_divO("/", 2, true, operatorResolve(genericOperatorTypes({INT,FLOAT}, {binaryDivO<int>, binaryDivO<float>}, 2)));
const Operator<ExpressionDesc> DataType_andO(".*&&.*", 2, true, operatorResolve(genericOperatorTypes({BOOLEAN}, {binaryAndO<bool>}, 2)));
const Operator<ExpressionDesc> DataType_multO(".*\\*.*", 2, true, operatorResolve(genericOperatorTypes({INT,FLOAT}, {binaryMultO<int>, binaryMultO<float>}, 2)));

const Operator<ExpressionDesc> DataType_subO(".*-.*", 2, true, operatorResolve(genericOperatorTypes({INT,FLOAT}, {binarySubO<int>, binarySubO<float>}, 2)));
const Operator<ExpressionDesc> DataType_orO(".*\\|\\|.*", 2, true, operatorResolve(genericOperatorTypes({BOOLEAN}, {binaryOrO<bool>}, 2)));
const Operator<ExpressionDesc> DataType_addO(".*\\+.*", 2, true, operatorResolve(genericOperatorTypes({INT,FLOAT}, {binaryAddO<int>, binaryAddO<float>}, 2)));

const Operator<ExpressionDesc> DataType_impliesO(".*=>.*", 2, true, operatorResolve(genericOperatorTypes({BOOLEAN}, {binaryImpliesO<bool>}, 2)));

const Operator<ExpressionDesc> DataType_equalsO(".*==.*", 2, true, operatorResolve(genericOperatorTypesOtherReturn({INT,FLOAT,BOOLEAN}, {binaryEqualsO<int>, binaryEqualsFLOAT_O<float>, binaryEqualsO<bool>}, BOOLEAN, 2)));


*/

// Unary operators
// Precedence N/A
template <typename T>
T negation(T x) {
    return -x;
}

template <typename T>
T boolNegation(T x) {
    return !x;
}


// Binary operators

// Exponentiation: 160
template <typename T>
T exp(T x, T y) {
    return (T)powf((float)x, (float)y);
}

// Multiplicative: 150
template <typename T>
T mult(T x, T y) {
    return x * y;
}

template <typename T>
T div(T x, T y) {
    return x / y;
}

template <typename T>
T mod(T x, T y) {
    return x % y;
}


// Additive: 140
template <typename T>
T add(T x, T y) {
    return x + y;
}

template <typename T>
T subtract(T x, T y) {
    return x - y;
}


// Cast: 132
template <typename T, typename U>
U as(T x) {
    return (U)x;
}




// Implication: 131
template <typename T>
T implies(T x, T y) {
    return !x || y;
}

// Comparative: 130
template <typename T>
bool equal(T x, T y) {
    return x == y;
}

template <typename T>
bool notEqual(T x, T y) {
    return x != y;
}

template <typename T>
bool leq(T x, T y) {
    return x <= y;
}

template <typename T>
bool geq(T x, T y) {
    return x >= y;
}

template <typename T>
bool lessThan(T x, T y) {
    return x < y;
}

template <typename T>
bool greaterThan(T x, T y) {
    return x > y;
}


// Conjuctive: 120
template <typename T>
T _and(T x, T y) {
    return x && y;
}

// Disjunctive: 110
template <typename T>
T _or (T x, T y) {
    return x || y;
}




template <typename T>
TypeSpecification generateTypeSpecificationUnaryGenericUniform(DataType t, function<T (T)> f) {

    TypeSpecification spec;
    spec.inputTypes = {t};
    spec.returnType = t;
    spec.func = [f] (std::vector<int> xs) {
        
        T retVal = f(*(T *)&xs[0]);
        
        int retInt;
        bzero(&retInt, sizeof(int));
        
        memcpy(&retInt, &retVal, sizeof(T));
        
        return retInt;
    };
    
    return spec;
}

template <typename T>
TypeSpecification generateTypeSpecificationBinaryGenericUniform(DataType t, function<T (T,T)> f) {
    
    TypeSpecification spec;
    spec.inputTypes = {t, t};
    spec.returnType = t;
    spec.func = [f] (std::vector<int> xs) {
        
        T retVal = f(*(T *)&xs[0], *(T *)&xs[1]);
        
        int retInt;
        bzero(&retInt, sizeof(int));
        
        memcpy(&retInt, &retVal, sizeof(T));
        
        return retInt;
    };
    
    return spec;
}

template <typename T, typename U>
TypeSpecification generateTypeSpecificationBinaryGenericUniformSpecificReturn(DataType t, DataType u, function<U (T,T)> f) {
    
    TypeSpecification spec;
    spec.inputTypes = {t, t};
    spec.returnType = u;
    spec.func = [f] (std::vector<int> xs) {
        
        T lhs = *(T *)&xs[0];
        T rhs = *(T *)&xs[1];
        
        U retVal = f(lhs, rhs);
        
        int retInt;
        bzero(&retInt, sizeof(int));
        
        memcpy(&retInt, &retVal, sizeof(U));
                
        return retInt;
    };
    
    return spec;
}

map<string, OperatorType> OperatorType::operatorTypeMemoization;

OperatorType OperatorType::lookupOperatorType(std::string opString) {
    
    OperatorType &opType = operatorTypeMemoization[opString];
    
    if(opType.operatorText.length() > 0) {
        return opType;
    }
    
    opType.isIdentityOperator = false;
    opType.operatorText = opString;
    
    int precedence;
    vector<TypeSpecification> typeSpecs;
    
    if(opString == "!") {
        TypeSpecification unaryNegationBool = generateTypeSpecificationUnaryGenericUniform<bool>(DataType::BOOLEAN, boolNegation<bool>);
        precedence = 999;
        typeSpecs = {unaryNegationBool};
    } else if(opString == "^") {
        TypeSpecification binaryInt = generateTypeSpecificationBinaryGenericUniform<int>(DataType::INT, exp<int>);
        TypeSpecification binaryFloat = generateTypeSpecificationBinaryGenericUniform<float>(DataType::FLOAT, exp<float>);
        
        typeSpecs = {binaryInt, binaryFloat};
        precedence = 160;
    } else if(opString == "*") {
        TypeSpecification binaryInt = generateTypeSpecificationBinaryGenericUniform<int>(DataType::INT, mult<int>);
        TypeSpecification binaryFloat = generateTypeSpecificationBinaryGenericUniform<float>(DataType::FLOAT, mult<float>);
        
        typeSpecs = {binaryInt, binaryFloat};
        precedence = 150;
    } else if(opString == "/") {
        TypeSpecification binaryInt = generateTypeSpecificationBinaryGenericUniform<int>(DataType::INT, div<int>);
        TypeSpecification binaryFloat = generateTypeSpecificationBinaryGenericUniform<float>(DataType::FLOAT, div<float>);
        
        typeSpecs = {binaryInt, binaryFloat};
        precedence = 150;
    } else if(opString == "%") {
        TypeSpecification binaryInt = generateTypeSpecificationBinaryGenericUniform<int>(DataType::INT, mod<int>);
        
        typeSpecs = {binaryInt};
        precedence = 150;
    } else if(opString == "+") {
        TypeSpecification binaryInt = generateTypeSpecificationBinaryGenericUniform<int>(DataType::INT, add<int>);
        TypeSpecification binaryFloat = generateTypeSpecificationBinaryGenericUniform<float>(DataType::FLOAT, add<float>);
        
        typeSpecs = {binaryInt, binaryFloat};
        precedence = 140;
    } else if(opString == "-") {
        TypeSpecification unaryMinusInt = generateTypeSpecificationUnaryGenericUniform<int>(DataType::INT, negation<int>);
        TypeSpecification unaryMinusFloat = generateTypeSpecificationUnaryGenericUniform<float>(DataType::FLOAT, negation<float>);

        TypeSpecification binaryMinusInt = generateTypeSpecificationBinaryGenericUniform<int>(DataType::INT, subtract<int>);
        TypeSpecification binaryMinusFloat = generateTypeSpecificationBinaryGenericUniform<float>(DataType::FLOAT, subtract<float>);

        typeSpecs = {unaryMinusInt, unaryMinusFloat, binaryMinusInt, binaryMinusFloat};
        precedence = 140;
    } else if(opString == "=>") {
        TypeSpecification binaryBool = generateTypeSpecificationBinaryGenericUniform<bool>(DataType::BOOLEAN, implies<bool>);
        
        typeSpecs = {binaryBool};
        precedence = 131;
    } else if(opString == "==") {
        TypeSpecification binaryBool = generateTypeSpecificationBinaryGenericUniformSpecificReturn<bool, bool>(DataType::BOOLEAN, DataType::BOOLEAN, equal<bool>);
        TypeSpecification binaryInt = generateTypeSpecificationBinaryGenericUniformSpecificReturn<int, bool>(DataType::INT, DataType::BOOLEAN, equal<int>);

        typeSpecs = {binaryBool, binaryInt};
        precedence = 130;
    } else if(opString == "!=") {
        TypeSpecification binaryBool = generateTypeSpecificationBinaryGenericUniformSpecificReturn<bool, bool>(DataType::BOOLEAN, DataType::BOOLEAN, notEqual<bool>);
        TypeSpecification binaryInt = generateTypeSpecificationBinaryGenericUniformSpecificReturn<int, bool>(DataType::INT, DataType::BOOLEAN, notEqual<int>);
        TypeSpecification binaryFloat = generateTypeSpecificationBinaryGenericUniformSpecificReturn<float, bool>(DataType::FLOAT, DataType::BOOLEAN, notEqual<float>);
        
        typeSpecs = {binaryBool, binaryInt, binaryFloat};
        precedence = 130;
    } else if(opString == "<=") {
        TypeSpecification binaryBool = generateTypeSpecificationBinaryGenericUniformSpecificReturn<bool, bool>(DataType::BOOLEAN, DataType::BOOLEAN, leq<bool>);
        TypeSpecification binaryInt = generateTypeSpecificationBinaryGenericUniformSpecificReturn<int, bool>(DataType::INT, DataType::BOOLEAN, leq<int>);
        TypeSpecification binaryFloat = generateTypeSpecificationBinaryGenericUniformSpecificReturn<float, bool>(DataType::FLOAT, DataType::BOOLEAN, leq<float>);
        
        typeSpecs = {binaryBool, binaryInt, binaryFloat};
        precedence = 130;
    } else if(opString == ">=") {
        TypeSpecification binaryBool = generateTypeSpecificationBinaryGenericUniformSpecificReturn<bool, bool>(DataType::BOOLEAN, DataType::BOOLEAN, geq<bool>);
        TypeSpecification binaryInt = generateTypeSpecificationBinaryGenericUniformSpecificReturn<int, bool>(DataType::INT, DataType::BOOLEAN, geq<int>);
        TypeSpecification binaryFloat = generateTypeSpecificationBinaryGenericUniformSpecificReturn<float, bool>(DataType::FLOAT, DataType::BOOLEAN, geq<float>);
        
        typeSpecs = {binaryBool, binaryInt, binaryFloat};
        precedence = 130;
    } else if(opString == "<") {
        TypeSpecification binaryBool = generateTypeSpecificationBinaryGenericUniformSpecificReturn<bool, bool>(DataType::BOOLEAN, DataType::BOOLEAN, lessThan<bool>);
        TypeSpecification binaryInt = generateTypeSpecificationBinaryGenericUniformSpecificReturn<int, bool>(DataType::INT, DataType::BOOLEAN, lessThan<int>);
        TypeSpecification binaryFloat = generateTypeSpecificationBinaryGenericUniformSpecificReturn<float, bool>(DataType::FLOAT, DataType::BOOLEAN, lessThan<float>);
        
        typeSpecs = {binaryBool, binaryInt, binaryFloat};
        precedence = 130;
    } else if(opString == ">") {
        TypeSpecification binaryBool = generateTypeSpecificationBinaryGenericUniformSpecificReturn<bool, bool>(DataType::BOOLEAN, DataType::BOOLEAN, greaterThan<bool>);
        TypeSpecification binaryInt = generateTypeSpecificationBinaryGenericUniformSpecificReturn<int, bool>(DataType::INT, DataType::BOOLEAN, greaterThan<int>);
        TypeSpecification binaryFloat = generateTypeSpecificationBinaryGenericUniformSpecificReturn<float, bool>(DataType::FLOAT, DataType::BOOLEAN, greaterThan<float>);
        
        typeSpecs = {binaryBool, binaryInt, binaryFloat};
        precedence = 130;
    } else if(opString == "&&") {
        TypeSpecification binaryBool = generateTypeSpecificationBinaryGenericUniform<bool>(DataType::BOOLEAN, _and<bool>);
        
        typeSpecs = {binaryBool};
        precedence = 120;
    } else if(opString == "||") {
        TypeSpecification binaryBool = generateTypeSpecificationBinaryGenericUniform<bool>(DataType::BOOLEAN, _or<bool>);
        
        typeSpecs = {binaryBool};
        precedence = 110;
    } else {
        throw "Unknown operator: " + opString;
    }
    
    opType.specifications = typeSpecs;
    opType.precedence = precedence;
    
    return opType;
}




OperatorType OperatorType::identity() {
    std::vector<TypeSpecification> specs;
    std::vector<lemonscript::DataType> types = {lemonscript::DataType::BOOLEAN, lemonscript::DataType::INT, lemonscript::DataType::FLOAT};
    for(auto it = types.begin(); it != types.end(); ++it) {
        lemonscript::DataType t = *it;
        TypeSpecification spec;
        spec.inputTypes = {t};
        spec.returnType = t;
        spec.func = [] (std::vector<int> xs) { return xs[0]; };
        specs.push_back(spec);
    }
    OperatorType opType;
    opType.isIdentityOperator = true;
    opType.specifications = specs;
    opType.operatorText = "i";
    opType.precedence = 999;
    
    return opType;
}
