#ifndef O2016_SUBSYSTEMS_SUPERSTRUCTURE_CATAPULT_H_
#define O2016_SUBSYSTEMS_SUPERSTRUCTURE_CATAPULT_H_

#include "stop/stop.h"
#include "scoop/scoop.h"
#include "catapult_messages.h"
#include "muan/units/units.h"
#include "muan/queues/message_queue.h"
#include <memory>

class Catapult {
public:
  Catapult(std::shared_ptr<muan::queues::MessageQueue<CatapultInput, 10>> input_queue,
           std::shared_ptr<muan::queues::MessageQueue<CatapultGoal, 10>> goal_queue,
           std::shared_ptr<muan::queues::MessageQueue<CatapultOutput, 10>> output_queue,
           std::shared_ptr<muan::queues::MessageQueue<CatapultStatus, 10>> status_queue);
  ~Catapult() = default;
  void Update();

protected:
  CatapultStop stop_;
  Scoop scoop_;
  bool cylinder_extended_;
  muan::queues::MessageQueue<CatapultInput, 10>::QueueReader input_;
  muan::queues::MessageQueue<CatapultGoal, 10>::QueueReader goal_;
  std::shared_ptr<muan::queues::MessageQueue<CatapultOutput, 10>> output_;
  std::shared_ptr<muan::queues::MessageQueue<CatapultStatus, 10>> status_;
};

#endif // O2016_SUBSYSTEMS_SUPERSTRUCTURE_CATAPULT_H_
