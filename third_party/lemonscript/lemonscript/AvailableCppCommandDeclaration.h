//
//  AvailableCppCommand.hpp
//  lemonscript
//
//  Created by Donald Pinckney on 12/24/15.
//  Copyright © 2015 Donald Pinckney. All rights reserved.
//

#ifndef AvailableCppCommand_hpp
#define AvailableCppCommand_hpp

#include <stdio.h>
#include <vector>
#include <string>
#include <string.h>
#include <strings.h>
#include <functional>
#include <map>
#include <memory>

#include "lemonscript.h"

namespace lemonscript {
    bool DataTypeIsSubtypeOf(DataType subtype, DataType type);
    
    int DataTypeIntCastFromTo(DataType from, DataType to, int value);

    template <typename T>
    int DataTypeBuildInt(T val) {
        int retVal;
        bzero(&retVal, sizeof(int));
        
        memcpy(&retVal, &val, sizeof(T));
        
        return retVal;
    }


}


std::string dataTypeDescription(lemonscript::DataType t);


class lemonscript::AvailableCppCommandDeclaration {

public:
    
    std::string functionName; // This function name has been camel-cased, and is in final form for C++
    std::function<std::unique_ptr<BaseAutoFunction>()> generatorFunction;
    std::vector<DataType> parameters; // in order from left to right
    
    
    AvailableCppCommandDeclaration(const std::function<std::unique_ptr<BaseAutoFunction>()> &genFunc, const std::string &name, const std::vector<DataType> &ps) : functionName(name), generatorFunction(genFunc), parameters(ps) { };
    
    // Make sure you deallocate these later!
    static std::vector<const AvailableCppCommandDeclaration *> parseCppCommands(const std::map<std::string, std::function<std::unique_ptr<BaseAutoFunction>()>> &autoFunctions);
};


#endif /* AvailableCppCommand_hpp */
