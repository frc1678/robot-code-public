/*
 * updateable.h
 *
 *  Created on: Dec 18, 2015
 *  Copyright 2015 Citrus Circuits
 *      Author: Kyle Stachowicz
 */

#ifndef MUAN_MULTITHREADING_UPDATEABLE_H_
#define MUAN_MULTITHREADING_UPDATEABLE_H_

#include "muan/units/units.h"
#include <atomic>
#include <thread>

namespace muan {

/*
 * A base class for an object that runs in a separate thread at a constant rate.
 * To use, implement the Update(Seconds dt) method.
 */
class Updateable {
 public:
  explicit Updateable(muan::units::Seconds tick_rate);
  ~Updateable();
  virtual void Update(muan::units::Seconds dt) = 0;
  void Start();
  void Stop();

 private:
  void RunForever();
  muan::units::Seconds loop_time;

  std::thread main_;
  volatile std::atomic<bool> running_{false};
};
}

#endif  // MUAN_MULTITHREADING_UPDATEABLE_H_
