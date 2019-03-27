#include "muan/lime/lime.h"
#include <memory>

using muan::queues::QueueManager;

namespace muan {
namespace lime {

Lime::Lime(double lime_height, double lime_angle, double object_height)
    : status_queue_{QueueManager<LimeStatusProto>::Fetch()},
      lime_height_(lime_height),
      lime_angle_(lime_angle),
      object_height_(object_height) {}

Lime::Lime(double lime_height, double lime_angle, double object_height,
           double dist_factor, double dist_offset)
    : status_queue_{QueueManager<LimeStatusProto>::Fetch()},
      lime_height_(lime_height),
      lime_angle_(lime_angle),
      object_height_(object_height),
      dist_factor_(dist_factor),
      dist_offset_(dist_offset) {}

void Lime::GetTable() {
  auto inst = nt::NetworkTableInstance::GetDefault();
  std::shared_ptr<nt::NetworkTable> table = inst.GetTable("limelight");
  target_horizontal_angle_ = table->GetEntry("tx").GetDouble(0);
  target_vertical_angle_ = table->GetEntry("ty").GetDouble(0);
  target_area_ = table->GetEntry("ta").GetDouble(0);
  target_skew_ = table->GetEntry("ts").GetDouble(0);
  target_present_ = table->GetEntry("tv").GetDouble(0);
}

double Lime::ObjectDistance(double vertical_angle) {
  distance_ = (lime_height_ - object_height_) *
              tan((M_PI / 180.) * (lime_angle_ + vertical_angle));
  return distance_;
}

void Lime::Update() {
  /* LimeStatusProto status; */
  /* LimeGoalProto goal; */

  /* if (goal_reader_.ReadLastMessage(&goal)) { */
  /*   auto inst = nt::NetworkTableInstance::GetDefault(); */
  /*   std::shared_ptr<nt::NetworkTable> table = inst.GetTable("limelight"); */
  /*   table->PutNumber("ledMode", static_cast<int>(goal->light_state())); */
  /* } */

  /* status->set_has_target(target_present_); */
  /* if (target_present_) { */
  /*   status->set_dist((ObjectDistance(target_vertical_angle_) * dist_factor_) - */
  /*                    dist_offset_); */
  /*   status->set_theta(target_horizontal_angle_); */
  /*   status->set_relative_x(std::cos(status->theta() * (M_PI / 180.)) * */
  /*                          status->dist()); */
  /*   status->set_relative_y(std::sin(status->theta() * (M_PI / 180.)) * */
  /*                          status->dist()); */
  /* } */
  /* status_queue_->WriteMessage(status); */
}

}  // namespace lime
}  // namespace muan
