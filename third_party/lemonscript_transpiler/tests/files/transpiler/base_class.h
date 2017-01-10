#ifndef BASE_CLASS_H_
#define BASE_CLASS_H_

class CitrusRobot;

namespace lemonscript {

class BaseAutoFunction {
  public:
    bool Init(std::vector<void *>);
    bool Periodic(std::vector<void *>);
  private:
    //none
};

}

#endif
