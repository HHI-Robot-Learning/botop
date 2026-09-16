#include "TrossenThread.h"

// The model's URDF was converted with joints 2/3/4 axis-flipped (PhysX cannot build a
// generic hinge from a negative axis), so the driver and the model disagree in sign on
// exactly those three. Convert on both boundaries so everything above TrossenThread —
// BotOp, sync(), the viewer, KOMO — works in model convention.
// TODO hardcoded indices; should come from the URDF (see wxai.prepare_model).
static void flipTrossenSigns(arr& q){
  if(q.N>4){ q(2)*=-1.; q(3)*=-1.; q(4)*=-1.; }
}

#ifdef RAI_TROSSEN

//COPY AND PASTE from trossen_arm/demos/cpp/gravity_compensation

#include "libtrossen_arm/trossen_arm.hpp"

TrossenThread::TrossenThread(rai::Var<rai::CtrlCmdMsg>& cmd, rai::Var<rai::CtrlStateMsg>& state, const char* ipAddress)
    : rai::RobotAbstraction(cmd, state),
    Thread("TrossenThread", .002), //HARD CODED step frequency of 100Hz
    ipAddress(ipAddress), fil("trossen.dat") {

  Kp = rai::getParameter<arr>("Trossen/Kp");
  Kd = rai::getParameter<arr>("Trossen/Kd"); //FOR TROSSEN, this corresponds to the Kp of the velocity PID

  LOG(0) <<"launching Trossen at " <<ipAddress;

  threadLoop(true);   // wait until open() has read q_init from hardware, otherwise
                      // step() runs with BotOp's qHome (the model pose) for a few
                      // ticks and commands the arm there
}

void print_motor_parameters(const std::vector<std::map<trossen_arm::Mode, trossen_arm::MotorParameter>>& motor_parameters)
{
  std::cout << "Motor parameters:" << std::endl;
  for (size_t i = 0; i < motor_parameters.size(); ++i) {
    const std::map<trossen_arm::Mode, trossen_arm::MotorParameter>& motor_parameter =
        motor_parameters.at(i);
    std::cout << "  Joint " << i << ":" << std::endl;
    for (const auto& [mode, parameter] : motor_parameter) {
      std::cout << "    Mode " << static_cast<int>(mode) << ":" << std::endl;
      std::cout << "      Position loop:";
      std::cout << " kp: " << parameter.position.kp;
      std::cout << ", ki: " << parameter.position.ki;
      std::cout << ", kd: " << parameter.position.kd;
      std::cout << ", imax: " << parameter.position.imax << std::endl;
      std::cout << "      Velocity loop:";
      std::cout << " kp: " << parameter.velocity.kp;
      std::cout << ", ki: " << parameter.velocity.ki;
      std::cout << ", kd: " << parameter.velocity.kd;
      std::cout << ", imax: " << parameter.velocity.imax << std::endl;
    }
  }
}

void TrossenThread::open(){
  driver = make_shared<trossen_arm::TrossenArmDriver>();

  driver->configure(
      trossen_arm::Model::wxai_v0,
      trossen_arm::StandardEndEffector::wxai_v0_follower,
      ipAddress.p,
      true
      );

  auto motor_parameters = driver->get_motor_parameters();
  print_motor_parameters(motor_parameters);

#if 0 //totally bad yet!!
  for(uint i=0;i<Kp.N;i++){
    motor_parameters.at(i).at(trossen_arm::Mode::position).position.kp = Kp(i);
    motor_parameters.at(i).at(trossen_arm::Mode::position).velocity.kp = Kd(i);
  }
  driver->set_motor_parameters(motor_parameters);
#else
  driver->set_motor_parameters(trossen_arm::StandardMotorParameters::wxai_v0_latest);

    // Experiment: the stock position loop has kd = 0, which rings at low speed.
    {
      double kd = rai::getParameter<double>("Trossen/motorKd", 0.);
      if(kd>0.){
        auto mp = driver->get_motor_parameters();
        for(auto& joint : mp){
          joint.at(trossen_arm::Mode::position).position.kd = kd;
        }
        driver->set_motor_parameters(mp);
        LOG(0) <<"Trossen: position-loop kd set to " <<kd;
      }
    }
#endif

  //get initial state
  arr q_init = as_arr(driver->get_all_positions(), false);
  flipTrossenSigns(q_init);
  {
    auto stateSet = state.set();
    stateSet->q = q_init;
    stateSet->qDot.resize(q_init.N).setZero();
    stateSet->tauExternalIntegral.resize(q_init.N).setZero();
    stateSet->tauExternalCount=0;
  }
  {
    auto cmd_set = cmd.set();
    cmd_set->setConst(q_init, false, true);
  }

  // start effort control mode
#if 0 //own PD
  driver->set_all_modes(trossen_arm::Mode::external_effort);
  driver->set_all_external_efforts({0, 0, 0, 0, 0, 0, 0}, 0.0f, false);
#else
  //TODO, set motor params according to Kp Kd - for now just defaults
  driver->set_all_modes(trossen_arm::Mode::position);
#endif
}

void TrossenThread::close(){
  if(!driver) return;
  try{
    driver->set_all_modes(trossen_arm::Mode::idle);
    rai::wait(.1);
  }catch(const std::exception& e){
    LOG(-1) <<"Trossen: could not set idle on close: " <<e.what();
  }
  driver.reset();
}

void TrossenThread::step(){
  try{

    //-- get real state
    arr q_real = as_arr(driver->get_all_positions(), false);
    arr qDot_real = as_arr(driver->get_all_velocities(), false);
    arr tauExternal = as_arr(driver->get_all_external_efforts(), false);

    flipTrossenSigns(q_real);
    flipTrossenSigns(qDot_real);
    flipTrossenSigns(tauExternal);

    //-- publish state & INCREMENT CTRL TIME
    {
      auto stateSet = state.set();
      if(!stateSet->stall) stateSet->ctrlTime += metronome.ticInterval;
      else stateSet->stall--;
      ctrlTime = stateSet->ctrlTime;
      stateSet->q = q_real;
      stateSet->qDot = qDot_real;
      stateSet->tauExternalIntegral += tauExternal;
      stateSet->tauExternalCount++;
    }

    // Contact detection by high-passing the external efforts: inertia and gravity vary
    // over seconds, contact arrives in milliseconds. tauSlow tracks the slow part, and
    // what is left over is the contact. Measured 2026-09-14: at moveTo speeds the raw
    // |tau| baseline is ~1.0 while moving, which is why a plain threshold on |tau| was
    // firing for most of the trajectory.
    {
      if(tauSlow.N != tauExternal.N){ tauSlow = tauExternal; }
      double tc = rai::getParameter<double>("Trossen/tauBaselineTime", .5);
      double alpha = metronome.ticInterval / tc;
      tauSlow += alpha * (tauExternal - tauSlow);

      double dev = 0.;
      for(uint i=0;i<6 && i<tauExternal.N;i++){
        double e = tauExternal(i) - tauSlow(i);
        dev += e*e;
      }
      dev = sqrt(dev);

      (void)dev;   // kept for the log only; contact is now detected by tracking error
    }

    //-- get current ctrl reference
    arr q_ref, qDot_ref, qDDot_ref;
    {
      auto cmdGet = cmd.get();

      //get commanded reference from the reference callback (e.g., sampling a spline reference)
      if(cmdGet->ref){
        cmdGet->ref->getReference(q_ref, qDot_ref, qDDot_ref, q_real, qDot_real, ctrlTime);
      }else{
        q_ref = q_real;
      }
    }

    //write into log file, need to be made optional
    fil <<ctrlTime <<' ' <<q_ref.modRaw() <<' ' <<q_real.modRaw() <<' ' <<tauExternal.modRaw() <<' ' <<tauSlow.modRaw() <<endl;

    //-- check reference error
    bool isStalled = false;
    if(q_ref.N){
      double err = length(q_ref - q_real);
      double stallThreshold = rai::getParameter<double>("Trossen/stallThreshold", .05);
      if(err>stallThreshold){ //stall!
        state.set()->stall = 2;
        isStalled=true;
        cout <<"STALLING - err: " <<err <<endl;
      }

      // Contact detection by tracking error. When something holds the arm back, q_real
      // falls behind q_ref and STAYS behind, because the spline reference keeps
      // advancing. Measured 2026-09-14: free motion ~0.018, hand contact ~0.045.
      // Unlike the high-passed external efforts, this does not adapt to a sustained
      // push — that was why the tau-based detector lost contact after ~0.15 s.
      double touchErr = rai::getParameter<double>("Trossen/touchErr", 0.);
      uint touchTicks = rai::getParameter<double>("Trossen/touchTicks", 20);
      bool baselineReady = (ctrlTime > 1.5);
      if(touchErr>0. && baselineReady && err>touchErr) touchCount++;
      else touchCount = 0;

      if(touchErr>0. && touchCount==touchTicks){
        LOG(0) <<"CONTACT: tracking error " <<err <<" for " <<touchTicks <<" ticks";
        state.set()->contact = true;
      }
    }

#if 0 //own PD
    arr u;
    u.resize(q_real.N).setZero();
    if(q_ref.N){
      u += Kp % (q_ref - q_real);
      u += Kd % (qDot_ref - qDot_real);
    }

    driver->set_all_external_efforts(as_vector(u), 0.0f, false);
#else
    if(q_ref.N){
      if(!isStalled){
        arr q_cmd = q_ref;
        flipTrossenSigns(q_cmd);
        arr qDot_cmd = qDot_ref;
        flipTrossenSigns(qDot_cmd);
        driver->set_all_positions(as_vector(q_cmd), 0.0f, false, as_vector(qDot_cmd));
      }
    }
#endif

  }catch(const std::exception& e){
    // The driver's daemon stores an exception and rethrows it on the next call, so a
    // network drop surfaces here. step() runs inside a thread, and an exception leaving
    // a thread is std::terminate() — which is why every driver error so far killed the
    // process before close() could run, leaving the controller refusing TCP until it
    // was power-cycled. Stop the thread cleanly instead.
    LOG(-1) <<"Trossen driver error in step(): " <<e.what() <<" -- stopping thread";
    threadStop();
  }
}

#else

TrossenThread::TrossenThread(rai::Var<rai::CtrlCmdMsg>& cmd, rai::Var<rai::CtrlStateMsg>& state, const char* ipAddress)
    : rai::RobotAbstraction(cmd, state),
    Thread("TrossenThread", .002), //HARD CODED step frequency of 100Hz
    ipAddress(ipAddress), fil("trossen.dat") { NICO }
void TrossenThread::open(){ NICO }
void TrossenThread::step(){ NICO }
void TrossenThread::close(){ NICO }

#endif
