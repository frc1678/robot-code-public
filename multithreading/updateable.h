/*
 * updateable.h
 *
 *  Created on: Dec 18, 2015
 *  Copyright 2015 Citrus Circuits
 *      Author: Kyle Stachowicz
 */

#ifndef MUAN_MULTITHREADING_UPDATEABLE_H_
#define MUAN_MULTITHREADING_UPDATEABLE_H_

#include <atomic>
#include <thread>
#include "../unitscpp/unitscpp.h"

namespace muan {

/*
 * A base class for an object that runs in a separate thread at a constant rate.
 * To use, implement the Update(Time dt) method.
 */
class Updateable {
 public:
  explicit Updateable(Frequency tick_rate);
  ~Updateable();
  virtual void Update(Time dt) = 0;
  void Start();
  void Stop();

 private:
  std::thread main_;
  volatile std::atomic<bool> running_{false};
};
}

#endif  // MUAN_MULTITHREADING_UPDATEABLE_H_
