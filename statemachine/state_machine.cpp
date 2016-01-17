/*
 * state_machine.cpp
 *
 *  Created on: Dec 7, 2015
 *  Copyright 2015 Citrus Circuits
 *      Author: Kyle Stachowicz
 */

#include "state_machine.h"
#include <string>
#include <iostream>

namespace muan {

StateMachine::StateMachine(std::string initial_state) : first_(initial_state) {}

void StateMachine::Start() {
  current_ = first_;
  if (states_.count(current_)) {
    states_[current_].Init();
  } else {
    std::cout << "No state found" << std::endl;
    // TODO(Kyle) Handle the case where the state does not exist
  }
}

void StateMachine::Update() {
  if (states_.count(current_)) {
    std::string next = states_[current_].Update();
    if (next != "") {
      if (states_.count(next)) {
        current_ = next;
        states_[current_].Init();
      }
    }
  } else {
    std::cout << "No state found" << std::endl;
    // TODO(Kyle) Handle the case where the state does not exist
  }
}

void StateMachine::AddState(State &to_add) { states_[to_add.Name()] = to_add; }

}
