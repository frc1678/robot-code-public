//
//  AvailableCppCommand.cpp
//  FiniteStateMachine
//
//  Created by Donald Pinckney on 12/24/15.
//  Copyright © 2015 Donald Pinckney. All rights reserved.
//

#include "AvailableCppCommandDeclaration.h"

#include <string.h>
#include <strings.h>

#include "BaseAutoFunction.h"
#include "ParsingUtils.h"

using lemonscript::DataType;

// This could be build into a more general tree, but whatever
bool lemonscript::DataTypeIsSubtypeOf(DataType subtype, DataType type) {
    if(subtype == DataType::TYPE) {
        return subtype == type;
    } else if(subtype == DataType::BOOLEAN) {
        return subtype == type;
    } else if(subtype == DataType::FLOAT) {
        return subtype == type;
    } else if(subtype == DataType::INT) {
        if (type == DataType::FLOAT) {
            return true;
        }
        return subtype == type;
    }
    
    throw "Unknown type";
}

int lemonscript::DataTypeIntCastFromTo(DataType from, DataType to, int value) {
    int retVal;
    bzero(&retVal, sizeof(int));
    
    if(from == to) {
        return value;
    } else if(from == DataType::BOOLEAN) {
        bool boolVal = *(bool *)(&value);
        
        if(to == DataType::INT) {
            int castValue = (int)boolVal;
            memcpy(&retVal, &castValue, sizeof(int));
        } else if(to == DataType::FLOAT) {
            float castValue = (float)boolVal;
            memcpy(&retVal, &castValue, sizeof(float));
        }
        
    } else if(from == DataType::INT) {
        int intVal = *(int *)(&value);
        
        if(to == DataType::BOOLEAN) {
            bool castValue = (bool)intVal;
            memcpy(&retVal, &castValue, sizeof(bool));
        } else if(to == DataType::FLOAT) {
            float castValue = (float)intVal;
            memcpy(&retVal, &castValue, sizeof(float));
        }
        
    } else if(from == DataType::FLOAT) {
        float floatVal = *(float *)(&value);
        
        if(to == DataType::INT) {
            int castValue = (int)floatVal;
            memcpy(&retVal, &castValue, sizeof(int));
        } else if(to == DataType::BOOLEAN) {
            bool castValue = (bool)floatVal;
            memcpy(&retVal, &castValue, sizeof(bool));
        }
    }
    
    return retVal;
}



std::string dataTypeDescription(DataType t) {
    switch (t) {
        case DataType::INT:
            return "INT";
        case DataType::FLOAT:
            return "FLOAT";
        case DataType::BOOLEAN:
            return "BOOLEAN";
        case DataType::TYPE:
            return "TYPE";
        case DataType::UNIT:
            return "UNIT";
    }
}


std::vector<const lemonscript::AvailableCppCommandDeclaration *> lemonscript::AvailableCppCommandDeclaration::parseCppCommands(const std::map<std::string, std::function<std::unique_ptr<BaseAutoFunction>()>> &autoFunctions) {
    
    std::vector<const AvailableCppCommandDeclaration *> decls;
    
    for(auto it = autoFunctions.begin(); it != autoFunctions.end(); it++) {
        std::string funcString = it->first;
        std::function<std::unique_ptr<BaseAutoFunction>()> func = it->second;
        
        std::vector<std::string> nameElems = ParsingUtils::split(funcString, '-');
        
        std::string funcName = nameElems[0];
        std::vector<std::string> typeStrings(nameElems.begin() + 1, nameElems.end());
        
        std::vector<DataType> types;
        for (auto typeIt = typeStrings.begin(); typeIt != typeStrings.end(); ++typeIt) {
            std::string ts = *typeIt;
            DataType t;
            if(ts == "int") {
                t = DataType::INT;
            } else if(ts == "float") {
                t = DataType::FLOAT;
            } else if(ts == "bool") {
                t = DataType::BOOLEAN;
            } else {
                t = DataType::UNIT;
            }
            types.push_back(t);
        }
        
        AvailableCppCommandDeclaration *decl = new AvailableCppCommandDeclaration(func, funcName, types);
        decls.push_back(decl);
    }
    
    return decls;
}
