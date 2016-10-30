#include "can_wrapper.h"

int main() {
  muan::wpilib::PdpWrapper::Queue pdp_queue;
  muan::wpilib::CanWrapper can{&pdp_queue};
}
