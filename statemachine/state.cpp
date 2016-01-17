/*
 * state.cpp
 *
 *  Created on: Dec 7, 2015
 *  Copyright 2015 Citrus Circuits
 *      Author: Kyle Stachowicz
 */

#include "state.h"
#include <string>

namespace muan {

State::State(std::string name, std::function<void()> init_func,
             std::function<std::string()> update_func)
    : name_(name), init_func_(init_func), update_func_(update_func) {}

State::State(std::string name, std::function<std::string()> update_func)
    : State(name, []() {}, update_func) {}

State::State(std::string name)
    : State(name, []() {}, []() -> std::string { return ""; }) {}

State::State() : State("Unnamed state") {}

State::~State() {}

void State::Init() { init_func_(); }

std::string State::Update() { return update_func_(); }

std::string State::Name() { return name_; }

}
