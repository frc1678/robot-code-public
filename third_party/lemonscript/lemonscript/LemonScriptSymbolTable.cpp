//
//  LemonScriptSymbolTable.cpp
//  lemonscript
//
//  Created by Donald Pinckney on 2/6/16.
//  Copyright © 2016 Donald Pinckney. All rights reserved.
//

#include "LemonScriptSymbolTable.h"

#include <stdlib.h>
#include <string.h>
#include <iostream>

using lemonscript::DataType;

std::vector<void *> LemonScriptSymbolTable::variableAddressHeap;

void LemonScriptSymbolTable::declareVariable(int line, const std::string &name, DataType type, void *pointerToValue) {
    if(variableAddresses.find(name) != variableAddresses.end()) {
        throw "Line " + std::to_string(line) + ": Duplicate variable definition: " + name;
    }
    
    void *address;
    switch (type) {
        case DataType::INT:
            address = malloc(sizeof(int));
            memcpy(address, pointerToValue, sizeof(int));
            break;
            
        case DataType::FLOAT:
            address = malloc(sizeof(float));
            memcpy(address, pointerToValue, sizeof(float));
            break;
            
        case DataType::BOOLEAN:
            address = malloc(sizeof(bool));
            memcpy(address, pointerToValue, sizeof(bool));
            break;
    }
    
    variableAddresses[name] = address;
    variableTypes[name] = type;
    
    variableAddressHeap.push_back(address);
}


void * LemonScriptSymbolTable::addressOfVariable(const std::string &variableName) const {
    auto it = variableAddresses.find(variableName);
    if(it != variableAddresses.end()) {
        return it->second;
    } else {
        return NULL;
    }
}

DataType LemonScriptSymbolTable::typeOfVariable(const std::string &variableName) const {
    auto it = variableTypes.find(variableName);

    if(it != variableTypes.end()) {
        return it->second;
    } else {
        return DataType::INT;
    }
}

void LemonScriptSymbolTable::freeVariables() {
    
    for (size_t i = 0; i < variableAddressHeap.size(); i++) {
        void *varAddress = variableAddressHeap[i];
        free(varAddress);
    }
    
    variableAddressHeap.clear();
}
