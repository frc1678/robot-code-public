//
//  lemonscript_tests.m
//  lemonscript_tests
//
//  Created by Donald Pinckney on 5/23/16.
//  Copyright © 2016 Donald Pinckney. All rights reserved.
//

#import <XCTest/XCTest.h>

#include <lemonscript/lemonscript.h>
#include <lemonscript/LemonScriptCompiler.h>
#include <string>

#include "PlayTestsShared.h"

#define XCTFailString(s) XCTFail("%s", s.c_str())

#define CAT_I(a,b) a##b
#define CAT(a,b) CAT_I(a, b)

#define STRINGIFY2(X) #X
#define STRINGIFY(X) STRINGIFY2(X)

#define make_test_compile(x) CAT(-(void)test_, x) { [self test_compile: @STRINGIFY(x) ".auto"]; }

using namespace lemonscript;

@interface lemonscript_tests : XCTestCase

@end

@implementation lemonscript_tests

- (void)setUp {
    [super setUp];
}

- (void)tearDown {
    [super tearDown];
}

// Helper Methods

- (std::string)auto_test_file_path:(NSString *)auto_name {
    NSBundle *bundle = [NSBundle bundleForClass:[self class]];
    NSString *path = [bundle pathForResource:auto_name ofType:@""];
    if(path == nil) {
        return "DID NOT LOAD FILE";
    }
    
    return std::string([path cStringUsingEncoding:NSUTF8StringEncoding]);
}

- (void)test_compile:(NSString *)file {
    try {
        [self measureMetrics:[[self class] defaultPerformanceMetrics] automaticallyStartMeasuring:NO forBlock:^{
            LemonScriptState *state = PlayTestsShared::play_tests_make_state();
            std::string fileName = [self auto_test_file_path:file];

            [self startMeasuring];
            lemonscript::LemonScriptCompiler *compiler = new lemonscript::LemonScriptCompiler(fileName, state);
            [self stopMeasuring];
            
            delete state;
            delete compiler;
        }];
    } catch (std::string err) {
        XCTFailString(err);
    }
}


// Tests

// Test to see that we can create C++ function bindings, and add them to a lemonscript state.
- (void)test_make_state {
    try {
        LemonScriptState *state = PlayTestsShared::play_tests_make_state();
        delete state;
    } catch (std::string err) {
        XCTFailString(err);
    }
}


//// Performance and correctness tests for compilation only.
//- (void)test_compile_pointturn {
//    [self test_compile:@"pointturn.auto"];
//}



@end
