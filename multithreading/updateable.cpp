/*
 * updateable.cpp
 *
 *  Created on: Dec 18, 2015
 *  Copyright 2015 Citrus Circuits
 *      Author: Kyle Stachowicz
 */

#include "updateable.h"
#include <chrono>
#include <iostream>
#include "../utils/timing_utils.h"

namespace muan {

Updateable::Updateable(Frequency tick_rate) {
  Time loop_time = 1 / tick_rate;
  main_ = std::thread([this, loop_time]() {
    // Wait for it to start
    while (!this->running_) {
      sleep_for(loop_time);
    }
    Time until = now() + loop_time;
    while (this->running_) {
      // Capture the current time, adding 5 milliseconds to ensure exact timing
      Update(loop_time);

      // Maintain timing
      sleep_until(until);
      until = now() + loop_time;
    }
  });
  main_.detach();
}

Updateable::~Updateable() { Stop(); }

void Updateable::Start() { running_ = true; }

void Updateable::Stop() { running_ = false; }
}
