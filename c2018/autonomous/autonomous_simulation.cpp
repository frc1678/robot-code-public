#include <fstream>
#include <string>
#include <thread>
#include <vector>
#include "third_party/frc971/control_loops/drivetrain/drivetrain_simulation.h"
#include "muan/queues/queue_manager.h"
#include "muan/utils/string_utils.h"
#include "c2018/autonomous/autonomous_runner.h"
#include "c2018/subsystems/drivetrain/drivetrain_base.h"
#include "c2018/subsystems/drivetrain/drivetrain_dog_motor_plant.h"

using c2018::drivetrain::GetDrivetrainConfig;
using c2018::drivetrain::MakeDrivetrainPlant;
using frc971::control_loops::drivetrain::GoalProto;
using frc971::control_loops::drivetrain::InputProto;
using frc971::control_loops::drivetrain::OutputProto;
using frc971::control_loops::drivetrain::StatusProto;
using frc971::control_loops::drivetrain::testing::SimulationRunner;
using muan::webdash::AutoSelectionProto;
using muan::webdash::WebDashQueueWrapper;
using muan::wpilib::DriverStationProto;
using muan::wpilib::GameSpecificStringProto;
using muan::queues::QueueManager;

class C2018AutoSimulation : public SimulationRunner {
 public:
  C2018AutoSimulation()
      : SimulationRunner(GetDrivetrainConfig(), MakeDrivetrainPlant()) {}

  void RunAuto(std::string mode, std::string gss) {
    c2018::autonomous::AutonomousBase::set_simulation();

    AutoSelectionProto auto_proto;
    auto_proto->set_auto_modes("LL:" + mode + ";LR:" + mode + ";RL:" + mode + ";RR:" + mode);
    WebDashQueueWrapper::GetInstance().auto_selection_queue().WriteMessage(auto_proto);

    DriverStationProto ds_proto;
    ds_proto->set_mode(RobotMode::AUTONOMOUS);
    QueueManager<DriverStationProto>::Fetch()->WriteMessage(ds_proto);

    GameSpecificStringProto gss_proto;
    gss_proto->set_code(gss);
    QueueManager<GameSpecificStringProto>::Fetch()->WriteMessage(gss_proto);

    enable_mode_ = RobotMode::AUTONOMOUS;

    // Make sure everything is in the right state before starting auto
    RunIteration();

    // Run for 15s of auto, then 2s disabled to make sure the thread finishes
    c2018::autonomous::AutonomousRunner autonomous;
    std::thread auto_thread(autonomous);
    RunForTime(std::chrono::seconds(15));
    ASSERT_TRUE(status_queue_->ReadLastMessage());
    ds_proto->set_mode(RobotMode::DISABLED);
    enable_mode_ = RobotMode::DISABLED;
    QueueManager<DriverStationProto>::Fetch()->WriteMessage(ds_proto);
    RunForTime(std::chrono::seconds(2));
    auto_thread.join();
  }
};

int main(int argc, char *argv[]) {
  if (argc != 3) {
    std::cerr << "Usage example: autonomous_simulation SCALE_PLUS_SWITCH RRR" << std::endl;
    return 1;
  }
  std::string name(argv[1]);
  std::string gss(argv[2]);
  std::string dirname = "/tmp/" + name + "_" + gss;
  std::string filename = dirname + "/status.csv";

  C2018AutoSimulation sim;
  sim.StartMocktime();
  sim.BeginLogging(dirname);
  sim.RunAuto(argv[1], argv[2]);

  std::ifstream csvfile(dirname + "/status.csv");
  std::string header;
  std::getline(csvfile, header);
  auto fields = muan::utils::split(header, ',');
  // Get column number of x and y position
  int x_goal_idx = -1, y_goal_idx = -1;
  for (size_t i = 0; i < fields.size(); i++) {
    if (fields[i] == "path_status.profiled_x_goal") {
      x_goal_idx = i + 1;
    }
    if (fields[i] == "path_status.profiled_y_goal") {
      y_goal_idx = i + 1;
    }
  }
  // set datafile separator "," # seperate columns by "," rather than " "
  // set term x11 # x11 graphics mode allows resizing in -p mode, default qt doesn't
  // set style data dots # data plots use small points, not large crosses (lines don't work in x11 mode)
  // using (-$y):x # plot x is robot -y (right), plot y is robot x (forward). This works if robot starts
  //     facing forward, otherwise a high-tech solution exists - rotate the laptop.
  //
  // Bazel sandboxing prevents directly calling system("gnuplot") since it isn't a standard program.
  // To work around this, send to stdout for use in "autonomous_simulation NAME GSS | sh".
  std::cout << "gnuplot -p -e \'set datafile separator \",\"; "
               "set term x11; set style data dots; "
               "plot \"" + filename + "\" using "
               "(-$" + std::to_string(y_goal_idx) + "):" + std::to_string(x_goal_idx) + "\'\n";
  return 0;
}
