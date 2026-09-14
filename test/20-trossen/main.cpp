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

  // Marc's hardcoded pose is for HIS arm — leaving it in would command ours to fly
  // there from wherever it stands:
  // q0 = {0.124552, 0.630388, 0.830282, -0.140574, -0.621233, 0.422866, 0.02};

  {
    BotOp bot(C, false);
    bot.launch_trossen();
    bot.wait(C, true, false);

    arr q_now, qDot_now; double t_now;
    bot.getState(q_now, qDot_now, t_now);
    cout <<"starting from: " <<q_now <<endl;

    uint T=5;
    arr path(T, q_now.N);
    for(uint t=0;t<T;t++){ path[t] = q_now; path(t,{0,5}) += 0.02*randn(5); } // play with number before *randn(5), tested: 0.02, with 0.1 and more crush!!!
    path[-1] = q_now;
    bot.move(path, {2.0});
    bot.wait(C);
  }

  // gnuplot("plot 'trossen.dat' us 1:4 t 'REF', '' us 1:11 t 'REAL'", true);
}


int main(int argc, char** argv){
  rai::initCmdLine(argc, argv);

    // direct();
    // thread();
    botop();
    return 0;
}
