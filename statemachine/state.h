/*
 * state.h
 *
 *  Created on: Dec 7, 2015
 *  Copyright 2015 Citrus Circuits
 *      Author: Kyle Stachowicz
 */

#ifndef MUAN_STATEMACHINE_STATE_H_
#define MUAN_STATEMACHINE_STATE_H_

#include <functional>
#include <string>

namespace muan {

class State {
 public:
  State();
  explicit State(std::string name);
  State(std::string name, std::function<std::string()> update_func);
  State(std::string name, std::function<void()> init_func,
        std::function<std::string()> update_func);
  virtual ~State();
  virtual void Init();
  virtual std::string Update();
  std::string Name();

 private:
  std::string name_;
  std::function<void()> init_func_;
  std::function<std::string()> update_func_;
};
}

#endif  // MUAN_STATEMACHINE_STATE_H_
