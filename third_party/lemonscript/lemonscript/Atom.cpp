//
//  Atom.cpp
//  lemonscript
//
//  Created by Donald Pinckney on 2/28/16.
//  Copyright © 2016 Donald Pinckney. All rights reserved.
//

#include "Atom.h"

using namespace lemonscript;

std::ostream & lemonscript_expressions::operator<<(std::ostream &o, const lemonscript_expressions::Atom &a) {
    
    o << "[";
    if(a.isIdentifier) {
        o << "ID";
    } else {
        o << dataTypeDescription(a.parsedType);
    }
    o << "]";
    
    o << a.text;
    
    return o;
}