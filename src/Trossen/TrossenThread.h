#include <Core/thread.h>
#include <Control/CtrlMsgs.h>

namespace trossen_arm{
class TrossenArmDriver;
}

struct TrossenThread : rai::RobotAbstraction, rai::Thread {
  std::shared_ptr<trossen_arm::TrossenArmDriver> driver;
  strA ipAddresses;
  strA endEffectors;
  arr Kp, Kd;
  double ctrlTime=0.;
  double stallThreshold=.05; //rai.cfg: Trossen/stallThreshold (default .05)
  bool failed=false;         //set when the driver threw; the thread then stops talking to this arm
  enum { torque_mode, position_mode } mode;

  ofstream dataFile;

  TrossenThread(rai::Var<rai::CtrlCmdMsg>& cmd, rai::Var<rai::CtrlStateMsg>& state, const strA& ids, const strA& ees={});
  ~TrossenThread(){
    LOG(0) <<"shutting down Trossen -- " <<timer.report();
    threadClose();
  }

  void open();
  void step();
  void step_inner();
  void close();
};
