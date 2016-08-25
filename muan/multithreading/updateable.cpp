/*
 * updateable.cpp
 *
 *  Created on: Dec 18, 2015
 *  Copyright 2015 Citrus Circuits
 *      Author: Kyle Stachowicz
 */

#include "updateable.h"
#include "muan/utils/timing_utils.h"
#include <chrono>
#include <iostream>

namespace muan {

Updateable::Updateable(Frequency tick_rate) : loop_time(1.0 / tick_rate) {}

void Updateable::RunForever() {
  Time until = now() + loop_time;
  while (this->running_) {
    // Capture the current time, adding 5 milliseconds to ensure exact timing
    Update(loop_time);

    // Maintain timing
    sleep_until(until);
    until = now() + loop_time;
  }
}

Updateable::~Updateable() { Stop(); }

void Updateable::Start() {
  running_ = true;
  main_ = std::thread(std::bind(&Updateable::RunForever, this));
  main_.detach();
}

void Updateable::Stop() { running_ = false; }
}
