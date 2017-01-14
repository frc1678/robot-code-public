//
//  CommentSanitizer.cpp
//  FiniteStateMachine
//
//  Created by Donald Pinckney on 1/16/16.
//  Copyright © 2016 Donald Pinckney. All rights reserved.
//

#include "ParsingUtils.h"

#include <sstream>

std::string ParsingUtils::removeCommentFromLine(const std::string &line) {
    std::string::size_type loc = line.find("//");
    
    if(loc == std::string::npos) {
        return line;
    }
    
    return line.substr(0, loc);
}

// From: http://stackoverflow.com/questions/10268872/c-fstream-function-that-reads-a-line-without-extracting
std::string ParsingUtils::peekLine(std::istream &is) {
    size_t len = is.tellg();

    std::string line;
    
    // Read line
    getline(is, line);
    
    // Return to position before "Read line".
    is.seekg(len , std::ios_base::beg);
    
    return line;
}

bool ParsingUtils::beginsWith(const std::string &x, const std::string &y) {
    return x.compare(0, y.length(), y) == 0;
}

bool ParsingUtils::isExecutableLine(const std::string &line) {
    for (auto it = line.begin(); it != line.end(); it++) {
        char c = *it;
        
        if(c == '/') {
            auto nextIt = it + 1;
            if (nextIt != line.end()) {
                char nextC = *nextIt;
                
                if(nextC == '/') {
                    return false;
                } else {
                    throw "Invalid token '/'";
                }
            } else {
                throw "Invalid token '/'";
            }
        }
        
        if(isalnum(c)) {
            return true;
        }
    }
    
    return false;
}

// From: http://stackoverflow.com/questions/3203452/how-to-read-entire-stream-into-a-stdstring
std::string ParsingUtils::readWholeStream(std::istream &in) {
    return std::string(std::istreambuf_iterator<char>(in), {});
}

// From: http://stackoverflow.com/questions/1494399/how-do-i-search-find-and-replace-in-a-standard-string
void findReplace(std::string& str, const std::string& oldStr, const std::string& newStr){
    size_t pos = 0;
    while((pos = str.find(oldStr, pos)) != std::string::npos){
        str.replace(pos, oldStr.length(), newStr);
        pos += newStr.length();
    }
}

std::string ParsingUtils::decreaseIndent(const std::string &s) {
    std::string result = "\n" + s;
    
    findReplace(result, "\n  ", "\n");
    
    return result.substr(1);
}

std::string ParsingUtils::trimWhitespace(const std::string &s) {
    size_t firstNotSpace = s.find_first_not_of(" ");
    size_t lastNotSpace = s.find_last_not_of(" ");
    return s.substr(firstNotSpace, lastNotSpace - firstNotSpace + 1);
    
}

std::vector<std::string> ParsingUtils::split(const std::string &s, char delim) {
    std::vector<std::string> elems;
    
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, delim)) {
        elems.push_back(item);
    }
    return elems;
}
