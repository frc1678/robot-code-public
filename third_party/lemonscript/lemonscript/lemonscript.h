//
//  lemonscript.h
//  lemonscript
//
//  Created by Donald Pinckney on 1/17/16.
//  Copyright © 2016 Donald Pinckney. All rights reserved.
//

#ifndef lemonscript_h
#define lemonscript_h

namespace lemonscript {
    class Command;
    class CppCommand;
    class SequentialCommand;
    class SimultaneousCommand;
    class WhileAlsoCommand;
    class CompleteAnyCommand;
    class CompleteCommand;
    class SetCommand;
    class DefCommand;
    class ImportCommand;
    class RunCommand;
    class OptionalCommand;
    class AvailableCppCommandDeclaration;
    class LemonScriptState;
    class LemonScriptTokenizer;
    class LemonScriptCompiler;
    class BaseAutoFunction;
    
    enum class DataType {
        INT, FLOAT, BOOLEAN, UNIT, TYPE
    };    
}

#endif /* lemonscript_h */
