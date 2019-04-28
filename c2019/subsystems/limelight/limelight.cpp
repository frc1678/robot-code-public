#include "c2019/subsystems/limelight/limelight.h"
#include <algorithm>
#include <memory>
#include <vector>
#include "c2019/subsystems/superstructure/queue_types.h"

using c2019::superstructure::SuperstructureStatusProto;
using muan::queues::QueueManager;

namespace c2019 {
namespace limelight {

Limelight::Limelight(const double limelight_height,
                     const double limelight_angle, const double object_height)
    : status_queue_{QueueManager<LimelightStatusProto>::Fetch()},
      limelight_height_(limelight_height),
      limelight_angle_(limelight_angle),
      object_height_(object_height) {}

void Limelight::operator()() {
  aos::time::PhasedLoop phased_loop(std::chrono::milliseconds(10));
  muan::utils::SetCurrentThreadName("Limelight");

  running_ = true;

  while (running_) {
    Update();
    BackUpdate();
    phased_loop.SleepUntilNext();
  }
}

void Limelight::Update() {
  LimelightStatusProto status;

  auto inst = nt::NetworkTableInstance::GetDefault();

  std::shared_ptr<nt::NetworkTable> table = inst.GetTable("limelight-front");
  std::shared_ptr<nt::NetworkTable> pricey_table =
      inst.GetTable("limelight-pricey");
  std::shared_ptr<nt::NetworkTable> back_table =
      inst.GetTable("limelight-back");

  double target_vertical_angle = table->GetEntry("ty").GetDouble(-1000);
  double skew = table->GetEntry("ts").GetDouble(-1000);
  double bottom_skew = pricey_table->GetEntry("ts").GetDouble(-1000);
  double target_horizontal_angle = table->GetEntry("tx").GetDouble(-1000);
  double has_target = table->GetEntry("tv").GetDouble(0);
  double latency = table->GetEntry("tl").GetDouble(-1000);

  double back_target_vertical_angle = back_table->GetEntry("ty").GetDouble(0);
  double back_target_horizontal_angle = back_table->GetEntry("tx").GetDouble(0);
  double back_latency = back_table->GetEntry("tl").GetDouble(-1000);

  double pricey_target_horizontal_angle =
      pricey_table->GetEntry("tx").GetDouble(0);
  double pricey_target_vertical_angle =
      pricey_table->GetEntry("ty").GetDouble(0);
  double bottom_latency = pricey_table->GetEntry("tl").GetDouble(-1000);

  if (latency == prev_latency_) {
    bad_ticks_++;
  } else {
    bad_ticks_ = 0;
  }

  if (back_latency == back_prev_latency_) {
    back_bad_ticks_++;
  } else {
    back_bad_ticks_ = 0;
  }

  if (bottom_latency == bottom_prev_latency_) {
    bottom_bad_ticks_++;
  } else {
    bottom_bad_ticks_ = 0;
  }
  prev_latency_ = latency;
  bottom_prev_latency_ = bottom_latency;
  back_prev_latency_ = back_latency;
  status->set_limelight_ok(bad_ticks_ < 10);
  status->set_back_limelight_ok(back_bad_ticks_ < 10);
  status->set_bottom_limelight_ok(bottom_bad_ticks_ < 10);

  // Very rough doesn't take horizontal translation
  target_dist_ =
      std::tan((target_vertical_angle + limelight_angle_) * (M_PI / 180.)) *
      ((limelight_height_ - object_height_) * 0.0254);

  horiz_angle_ = (target_horizontal_angle * (M_PI / 180.));

  if (skew > -45) {
    heading_ = std::abs(skew / 8.);
    to_the_left_ = true;
  } else {
    heading_ = ((skew + 90) / 8.);
    to_the_left_ = false;
  }

  bottom_to_the_left_ = (bottom_skew > -45);

  status->set_to_the_left(to_the_left_);
  status->set_bottom_skew(bottom_skew);
  status->set_bottom_to_the_left(bottom_to_the_left_);

  status->set_target_dist(target_dist_);
  status->set_skew(skew);
  status->set_has_target(has_target == 1);
  status->set_horiz_angle(horiz_angle_);

  // Very rough doesn't take horizontal translation
  back_target_dist_ =
      std::tan((back_target_vertical_angle + 60.0) * (M_PI / 180.)) *
      ((limelight_height_ - object_height_) * 0.0254);
  back_horiz_angle_ = (back_target_horizontal_angle * (M_PI / 180.));
  status->set_back_horiz_angle(back_horiz_angle_);
  status->set_back_target_dist(back_target_dist_);
  status->set_back_has_target(
      static_cast<bool>(back_table->GetEntry("tv").GetDouble(0)));
  pricey_horiz_angle_ = (pricey_target_horizontal_angle * (M_PI / 180.));

  status->set_pricey_horiz_angle(pricey_horiz_angle_);

  // Very rough doesn't take horizontal translation
  pricey_target_dist_ =
      ((object_height_)*0.0254) /
      std::tan((pricey_target_vertical_angle + 30.0) * (M_PI / 180.));

  status->set_pricey_horiz_angle(pricey_horiz_angle_);
  status->set_pricey_target_dist(pricey_target_dist_);
  status->set_pricey_has_target(
      static_cast<bool>(pricey_table->GetEntry("tv").GetDouble(0)));
  status_queue_->WriteMessage(status);
}

void Limelight::BackUpdate() { LimelightStatusProto status; }

}  // namespace limelight
}  // namespace c2019
