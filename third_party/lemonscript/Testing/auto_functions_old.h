//
//  auto_functions.h
//  FiniteStateMachine
//
//  Created by Donald Pinckney on 12/24/15.
//  Copyright © 2015 Donald Pinckney. All rights reserved.
//

#ifndef auto_functions_h
#define auto_functions_h

#include <stdio.h>

bool DeployChokehold(void *data);
bool DriveStraight_2(void *data, float speed, int distance);
bool DriveStraight_4(void *data, float speed, float distance);
bool DriveStraight_3(void *data, float speed, int distance, bool left);

bool CalibrateElevator(void *data);
bool StopElevator(void *data);
bool StopDriving(void *data);
bool RaiseWings(void *data);

bool TestBool(void *data, bool willPrintMe, bool otherBool);

bool PointTurn(void *data, float x, float y, bool highgear);
bool PointTurn2(void *data, float angle);
bool Wait(void *data, int t);
bool Wait5(void *data);

bool PrintF(void *data, float f);
bool PrintI(void *data, int i);
bool PrintB(void *data, bool b);

bool TestBool(void *data, bool willPrintMe, bool otherBool);

#endif /* auto_functions_h */
