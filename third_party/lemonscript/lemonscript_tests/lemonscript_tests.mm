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
#include "StdCapture.h"
#include <fstream>

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

- (void)test_run:(NSString *)file {
    try {
        LemonScriptState *state = PlayTestsShared::play_tests_make_state();
        std::string workingDir([[file stringByDeletingLastPathComponent] cStringUsingEncoding:NSASCIIStringEncoding]);
        state->setSearchPath(workingDir);
        
        NSString *autoName = [file lastPathComponent];
        std::string fileName = std::string([autoName cStringUsingEncoding:NSASCIIStringEncoding]);
        if(fileName == "") {
            NSLog(@"Error loading auto file: %@", file);
            XCTFail(@"File not found");
        }
        
        NSString *correctName = [file stringByAppendingString:@".correct"];
        
        NSError *err = nil;
        NSString *correct = [[NSString alloc] initWithContentsOfFile:correctName encoding:NSUTF8StringEncoding error:&err];
        if(err != nil) {
            NSLog(@"Error loading correct file: %@", correctName);
//            XCTFail(@"FIle not found");
        }
        
        StdCapture::BeginCapture();
        
        PlayTestsShared::run_at_path(fileName, state);
        
        StdCapture::EndCapture();
        std::string cap = StdCapture::GetCapture();
        NSString *strCap = [[NSString alloc] initWithCString:cap.c_str() encoding:NSUTF8StringEncoding];

        XCTAssertEqualObjects(strCap, correct);
        
        if(err != nil) {
            [strCap writeToFile:[NSString stringWithFormat:@"/Users/donaldpinckney/Stack/%@.correct", autoName] atomically:YES encoding:NSUTF8StringEncoding error:nil];
        }
    } catch (std::string err) {
        XCTFailString(err);
    }
}

- (void)testAll {
    NSBundle *bundle = [NSBundle bundleForClass:[self class]];
    NSArray *paths = [bundle pathsForResourcesOfType:@"auto" inDirectory:nil];
    for(NSString *path in paths) {
        [self test_run:path];
    }
}




//// Performance and correctness tests for compilation only.
//- (void)test_compile_pointturn {
//    [self test_compile:@"pointturn.auto"];
//}



@end
