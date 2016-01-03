#include "../statemachine/state_machine.h"
#include <iostream>
int i = 0;
int j = 0;
class TestStateMachine : public StateMachine {
 public:
  TestStateMachine() : StateMachine("s1") {
    State s1("s1", [&]() -> std::string {
      std::cout << i++ << std::endl;
      if (i > 100 + 10 * j) {
        return "s2";
      } else {
        return "s1";
      }
    });

    State s2("s2", [&j]() { j = 0; },
             [i, &j]() -> std::string {
               j++;
               if (j < 10) {
                 return "";
               } else if (i < 300) {
                 return "s1";
               } else {
                 return "s3";
               }
             });

    State s3("s3", [i]() { std::cout << "Done" << std::endl; },
             [i]() -> std::string {
               if (i == 300) {
                 exit(0);
               } else {
                 exit(1);
               }
             });

    AddState(s1);
    AddState(s2);
    AddState(s3);
  }
};

int main() {
  TestStateMachine tsm;
  tsm.Start();
  for (int k = 0; k < 10000; k++) {
    tsm.Update();
    // std::cout << "Doing things" << std::endl;
  }
}
