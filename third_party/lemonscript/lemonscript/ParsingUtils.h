//
//  CommentSanitizer.hpp
//  FiniteStateMachine
//
//  Created by Donald Pinckney on 1/16/16.
//  Copyright © 2016 Donald Pinckney. All rights reserved.
//

#ifndef CommentSanitizer_hpp
#define CommentSanitizer_hpp

#include <stdio.h>
#include <string>
#include <iostream>
#include <vector>

#include "lemonscript.h"

namespace ParsingUtils {
    std::string removeCommentFromLine(const std::string &line);
    std::string peekLine(std::istream &is);

    bool beginsWith(const std::string &x, const std::string &y);
    bool isExecutableLine(const std::string &line);
    
    std::string readWholeStream(std::istream &in);
    std::string decreaseIndent(const std::string &s);
    
    std::string trimWhitespace(const std::string &s);

    std::vector<std::string> split(const std::string &s, char delim);
}

#endif /* CommentSanitizer_hpp */
