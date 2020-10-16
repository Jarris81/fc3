#pragma once

#include <Kin/simulation.h>

#include <Control/control.h>
#include <Control/CtrlSet.h>

//just a workspace/playground to test controller chains
struct EndlessPicker{
  rai::Configuration RealWorld;
  std::shared_ptr<rai::Simulation> S;
  rai::Configuration C;

  std::shared_ptr<CtrlProblem> ctrl;
  rai::Array<std::shared_ptr<CtrlObjective>> qCosts;

  double tau = .01;
  Metronome tic;

  EndlessPicker() : tic(tau) {}
  ~EndlessPicker();

  //helpers to setup the scenario
  void setupSim(bool withImp);
  void setupC();
  void setupCtrl();
  void addSimObject();

  //standard control iteration: the control set determines everything
  void step();

  //preliminary test
  void pickAndLiftScript(const char* gripper, const char* gripperCenter, const char* object);

  //core test
  void pickAndLiftBehavior(const char* gripper, const char* gripperCenter, const char* object);
};
