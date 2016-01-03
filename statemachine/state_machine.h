/*
 * state_machine.h
 *
 *  Created on: Dec 7, 2015
 *  Copyright 2015 Citrus Circuits
 *      Author: Kyle Stachowicz
 */

#ifndef MUAN_STATEMACHINE_STATE_MACHINE_H_
#define MUAN_STATEMACHINE_STATE_MACHINE_H_

#include <string>
#include <map>
#include "state.h"

class StateMachine {
 public:
  explicit StateMachine(std::string initial_state);
  void Start();
  void Update();

 protected:
  void AddState(State &to_add);

 private:
  std::map<std::string, State> states_;
  std::string first_, current_;
};

#endif  // MUAN_STATEMACHINE_STATE_MACHINE_H_
