#include <Core/thread.h>
#include <Control/CtrlMsgs.h>

namespace trossen_arm{
class TrossenArmDriver;
}

struct TrossenThread : rai::RobotAbstraction, rai::Thread {
  std::shared_ptr<trossen_arm::TrossenArmDriver> driver;
  str ipAddress;
  arr Kp, Kd;
  arr tauSlow;   // slow-moving baseline of external efforts, for contact detection
  uint touchCount=0;   // consecutive ticks above the contact threshold
  double ctrlTime=0.;

  ofstream fil;

  TrossenThread(rai::Var<rai::CtrlCmdMsg>& cmd, rai::Var<rai::CtrlStateMsg>& state, const char* ipAddress="192.168.1.5");
  ~TrossenThread(){
    LOG(0) <<"shutting down Trossen -- " <<timer.report();
    threadClose();
  }

  void open();
  void step();
  void close();
};
