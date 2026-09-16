#include <fstream>
#include <Core/util.h>
#include <Core/thread.h>
#include <Control/CtrlMsgs.h>
#include <Kin/kin.h>
#include <BotOp/bot.h>

#include <Trossen/TrossenThread.h>

#ifdef RAI_TROSSEN

//COPY AND PASTE from trossen_arm/demos/cpp/gravity_compensation

#include <iostream>

#include "libtrossen_arm/trossen_arm.hpp"

int direct(){
  // Initialize the driver
  // trossen_arm::TrossenArmDriver driver;
  auto driver = std::make_shared<trossen_arm::TrossenArmDriver>();

  // Configure the driver
  driver->configure(
      trossen_arm::Model::wxai_v0,
      trossen_arm::StandardEndEffector::wxai_v0_follower, //changed for our arm till table arrived
      "192.168.1.5", // follower left (.3 was for leader left)
      true
      );

  // Start gravity compensation
  driver->set_all_modes(trossen_arm::Mode::external_effort);
  driver->set_all_external_efforts({0, 0, 0, 0, 0, 0, 0}, 0.0f, false);

  std::ofstream fil("direct.dat");
  double t=0.;
  for(uint k=0; k<2500; k++){        // 30 s at 500 Hz   // for friday tests switched from k<15000 to k<2500 (5s)
    auto q = driver->get_all_positions();
    fil <<t;
    for(auto v:q) fil <<' ' <<v;
    fil <<std::endl;
    if(!(k%250)) { cout <<"t=" <<t <<"  q:"; for(auto v:q) cout <<' ' <<v; cout <<endl; }
    t += .002;
    rai::wait(.002);
  }
  
  driver->set_all_modes(trossen_arm::Mode::idle);
  rai::wait(.5);
  driver.reset();

  return 0;
}

#endif


void thread(){
  rai::Var<rai::CtrlCmdMsg> cmd;
  rai::Var<rai::CtrlStateMsg> state;

  rai::Configuration C;
  C.addFile("scene.yml");

  TrossenThread trossen(cmd, state);

  for(;;){
    rai::wait(.02);
    arr q = state.get()->q;
    cout <<"q: " <<q <<endl;
    C.setJointState(q);
    int key = C.view(false);
    if(key=='q') break;
  }
}

void botop(){
  rai::Configuration C;
  C.addFile("scene.yml");
  arr q0 = C.getJointState();

  {
    BotOp bot(C, false);
    bot.launch_trossen();
    bot.wait(C, true, false);

    // Marc's hardcoded start pose, with joints 2/3/4 negated into our model's
    // convention. All seven values land inside the joint limits this way, which is
    // itself evidence that his scene used the driver's sign convention.
    // We commented this line out on Friday to stop the arm flying there from wherever
    // it stood — but that also started the random walk right at joint_1's lower
    // limit, which is what made it stall. Drive there first instead.
    q0 = {0.124552, 0.630388, -0.830282, 0.140574, 0.621233, 0.422866, 0.020000};
    cout <<"moving to Marc's start pose first" <<endl;
    bot.moveTo(q0, 1.);
    bot.wait(C);

    uint T=10;
    arr path(T, q0.N);
    for(uint t=0;t<T;t++){ path[t] = q0; path(t,{0,6}) += 0.3*randn(6); }
    path[-1] = q0;
    bot.move(path, {.5*T});
    bot.wait(C);
  }
}

void moveToTarget(){
  rai::Configuration C;
  C.addFile("scene.yml");

  {
    BotOp bot(C, false);
    bot.launch_trossen();
    bot.wait(C, true, false);

    arr q_now, qDot_now; double t_now;
    bot.getState(q_now, qDot_now, t_now);
    cout <<"starting from: " <<q_now <<endl;

    arr q_target = q_now;
    q_target(0) = 1.5;
    q_target(1) = 0.5;
    q_target(2) = -0.6;
    q_target(3) = 0.4;
    q_target(6) = 0.03;   // carriage travel in metres, range 0..0.044

    // works well
    /*arr q_target = q_now;
    q_target(0) += 0.3;     // base
    q_target(1) += 0.6;     // shoulder
    cout <<"target:        " <<q_target <<endl;*/

    // suspended, but only commented (for future debugs? idk)
    /* arr path(2, q_now.N);
    path[0] = q_now;
    path[1] = q_target;
    bot.move(path, {4.0});
    bot.wait(C); */

    bot.moveTo(q_target, 2.);

    // wait until either the move finishes or something touches the arm
    while(bot.getTimeToEnd()>0. && !bot.state.get()->contact){
      bot.sync(C, .01);
    }
    if(bot.state.get()->contact){
      cout <<"contact — stopping gently" <<endl;
      arr q_stop, qDot_stop; double t_stop;
      bot.getState(q_stop, qDot_stop, t_stop);
      bot.moveTo(q_stop, 3., true);   // overwrite=true: blend from current velocity
      bot.wait(C);
      rai::wait(1.5);
      bot.state.set()->contact = false;
    }

    bot.moveTo(q_now, 1.);
    bot.wait(C);

    cout <<"returned to start" <<endl;
  }
}


int main(int argc, char** argv){
  rai::initCmdLine(argc, argv);

  rai::String mode = rai::getParameter<rai::String>("mode", "moveToTarget");

  if(mode=="direct")           direct();        // gravity compensation, arm goes soft, logs direct.dat
  else if(mode=="thread")      thread();        // position mode, holds pose, reads state
  else if(mode=="botop")       botop();         // Marc's random offsets around the current pose
  else if(mode=="moveToTarget") moveToTarget(); // drive to a chosen configuration, stop on contact
  else{
    LOG(-1) <<"unknown mode '" <<mode <<"' -- use direct | thread | botop | moveToTarget";
    return 1;
  }
  return 0;
}