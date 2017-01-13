//
//  Atom.hpp
//  lemonscript
//
//  Created by Donald Pinckney on 2/28/16.
//  Copyright © 2016 Donald Pinckney. All rights reserved.
//

#ifndef Atom_hpp
#define Atom_hpp

#include <stdio.h>
#include <string>

#include "expressions.h"

#include "AvailableCppCommandDeclaration.h"

class lemonscript_expressions::Atom {
    
public:
    bool isIdentifier;
    std::string text;
    lemonscript::DataType parsedType; // Meaningless if atom is an identifier
};


#endif /* Atom_hpp */
