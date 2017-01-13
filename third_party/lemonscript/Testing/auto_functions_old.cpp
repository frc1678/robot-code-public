//
//  auto_functions.c
//  FiniteStateMachine
//
//  Created by Donald Pinckney on 12/24/15.
//  Copyright © 2015 Donald Pinckney. All rights reserved.
//

#include "auto_functions.h"

#include <time.h>

bool DeployChokehold(void *data) {
    
    printf("BANG! Chokehold has been deployed!\n");
    
    // Code here to deploy mechanism
    
    return true;
    
}


int currentDistance = 0;
bool DriveStraight_3(void *data, float speed, int distance, bool left) {
    
    
    printf("DriveStraight_3: Driving kinda straight, with speed = %f, distance = %d, left = %d?\n", speed, distance, left);
    
    
    // Simulate driving here
    currentDistance += 10; // in.
    
    if(currentDistance > distance) {
        currentDistance = 0;
        return true;
    }
    
    return false;
}

bool DriveStraight_2(void *data, float speed, int distance) {
    
    printf("DriveStraight_2: Driving kinda straight, with speed = %f, distance = %d?\n", speed, distance);
    
    
    // Simulate driving here
    currentDistance += 10; // in.
    
    if(currentDistance > distance) {
        currentDistance = 0;
        return true;
    }
    
    return false;
}

bool DriveStraight_4(void *data, float speed, float distance) {
    
    printf("DriveStraight_4: Driving kinda straight, with speed = %f, distance = %f?\n", speed, distance);
    
    
    // Simulate driving here
    currentDistance += 10; // in.
    
    if(currentDistance > distance) {
        currentDistance = 0;
        return true;
    }
    
    return false;
}

bool CalibrateElevator(void *data) {
    printf("Calibrating elevator...\n");
    
    return true;
}

bool StopElevator(void *data) {
    printf("Elevator STOP\n");
    return true;
}
bool StopDriving(void *data) {
    printf("Driving STOP\n");
    return true;
}

bool RaiseWings(void *data) {
    printf("Raising wings\n");
    return true;
}

bool PointTurn(void *data, float x, float y, bool highgear) {
    printf("%f, %f, %d\n", x, y, highgear);
    return true;
}

bool PointTurn2(void *data, float angle) {
    printf("%f\n", angle);
    return true;
}

bool TestBool(void *data, bool willPrintMe, bool otherBool) {
    printf("willPrintMe = %d, %d\n", willPrintMe, otherBool);
    return true;
}


int startIt = 0;
bool Wait(void *data, int t) {
    printf("Wait: %d\n", t);
    
    if(startIt >= t) {
        startIt = 0;
        return true;
    }
    startIt++;
    
    return false;
}

bool Wait5(void *data) {
    return Wait(data, 5);
}

bool PrintF(void *data, float f) {
    printf("f: %g\n", f);
    return true;
}

bool PrintI(void *data, int i) {
    printf("i: %d\n", i);
    return true;
}

bool PrintB(void *data, bool b) {
    printf("b: %s\n", b ? "true" : "false");
    return true;
}