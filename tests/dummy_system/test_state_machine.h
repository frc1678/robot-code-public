#include "../../statemachine/state_machine.h"
#include "unitscpp/unitscpp.h"

class TestStateMachine : public StateMachine {
 public:
  TestStateMachine(TestSubsystem& sub) : StateMachine("counting") {
    State countdown("counting", [=]() { this->i = 500; },
                    [=]() {
                      this->i--;
                      if (this->i <= 0) {
                        return "actuate";
                      } else {
                        return "";
                      }
                    });

    State actuate("actuate", [&]() { sub.Actuate(); },
                  []() { return "counting"; });

    AddState(countdown);
    AddState(actuate);
  }

 public:
  int i;
};
