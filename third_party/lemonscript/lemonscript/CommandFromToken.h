//
//  CommandFromToken.hpp
//  lemonscript
//
//  Created by Donald Pinckney on 8/26/16.
//  Copyright © 2016 Donald Pinckney. All rights reserved.
//

#ifndef CommandFromToken_hpp
#define CommandFromToken_hpp

#include <stdio.h>
#include "Command.h"
#include "LemonScriptTokenizer.h"

namespace lemonscript {
    lemonscript::Command *commandFromToken(const std::string &token, TokenType type, lemonscript::LemonScriptState *state, int lineNum);
}

#endif /* CommandFromToken_hpp */
