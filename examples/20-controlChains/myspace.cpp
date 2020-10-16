#include "myspace.h"
#include "mycontrollers.h"

//===========================================================================

EndlessPicker::~EndlessPicker(){
  qCosts.clear();
}

void EndlessPicker::setupSim(bool withImp){
  RealWorld.addFile("../../rai-robotModels/scenarios/pandasTable.g");

  rai::Frame *realObj = RealWorld.addFrame("obj0");
  realObj->setMass(1.);
  realObj->setColor({1.,0,0}); //set the color of one objet to red!
  realObj->setShape(rai::ST_sphere, {.03});
//      realObj->setShape(rai::ST_ssBox, {.06, .06, .15, .01});
  realObj->setPosition({0., .2, 2.});
  realObj->setContact(1);
  realObj->addAttribute("friction", 1.);

  S = make_shared<rai::Simulation>(RealWorld, S->_physx, 2);
  S->cameraview().addSensor("camera");

  //add imps!!
  if(withImp) S->addImp(S->_objectImpulses, {"obj0"}, {});
  //    S->addImp(S->_blockJoints, {"R_finger1", "L_finger1"}, {});
}

//===========================================================================

void EndlessPicker::setupC(){
  //-- setup your model world
  C.addFile("../../rai-robotModels/scenarios/pandasTable.g");

  rai::Frame *obj = C.addFrame("object");
  obj->setColor({1.,1.,0}); //set the color of one objet to red!
  obj->setShape(rai::ST_sphere, {.03});
  //    obj->setShape(rai::ST_ssBox, {.05,.05,.05,.01});

  C.watch(false, "model world start state");
}

//===========================================================================

void EndlessPicker::setupCtrl(){
  ctrl = make_shared<CtrlProblem>(C, tau, 2);

  auto qVel = ctrl->add_qControlObjective(1, 1e-1);
  auto qAcc = ctrl->add_qControlObjective(2, 1e-3);
  qCosts = {qVel, qAcc};
}

//===========================================================================

void EndlessPicker::addSimObject(){

  rai::Frame *realObj = RealWorld.addFrame("obj2");

  rai::Transformation pose;
  pose.setRandom();
  pose.pos.y *= .3;
  pose.pos.y += .5;
  pose.pos.z += 2.;
  realObj->setPose(pose);

  arr size = {rnd.uni(.03,.2), rnd.uni(.03,.2), rnd.uni(.03,.2), .01};
  realObj->setShape(rai::ST_ssBox, size);

  realObj->setMass(.2);
  realObj->setColor({1.,0,0}); //set the color of one objet to red!
  realObj->setContact(1);

  S->registerNewObjectWithEngine(realObj);

}

//===========================================================================

void EndlessPicker::step(){
  tic.waitForTic();

  ctrl->report();

  //TOTAL CHEAT: grab the true position from the RealWorld
  arr objectPose = RealWorld["obj0"]->getPose();
  //set the model object to percept
  C["object"]->setPose(objectPose);

  C.setJointState(S->get_q());
//  V.setConfiguration(C);

  ctrl->update(C);
  arr q = ctrl->solve();

  //send controls to the simulation
  uint subSteps=5;
  for(uint i=0;i<subSteps;i++){
    S->step(q, tau/subSteps, S->_position);
  }
}

//===========================================================================

void EndlessPicker::pickAndLiftScript(const char* gripper, const char* gripperCenter, const char* object){
  tic.reset(tau);
  bool verbose=false;

  CtrlSet PREPREGRASP = symbol2CtrlSet({"prePreGrasp", gripper, object}, ctrl->komo.world);
  CtrlSet PREGRASP = symbol2CtrlSet({"preGrasp", gripper, object}, ctrl->komo.world);
  CtrlSet GRASP = symbol2CtrlSet({"grasp", gripper, object}, ctrl->komo.world);
  CtrlSet LIFT = symbol2CtrlSet({"lift", gripper}, ctrl->komo.world);
  CtrlSet BOX = symbol2CtrlSet({"box", gripper, "R_workspace"}, ctrl->komo.world);

  S->openGripper(gripper, .07, 3.);
  ctrl->addObjectives(PREPREGRASP.objectives);

  while(!isFeasible(PREGRASP, ctrl->komo.pathConfig, true, 1e-3)) step();
  if(verbose){ cout <<"PREGRASP is feasible" <<endl;  rai::wait();  tic.reset(tau); }

  ctrl->delObjectives(PREPREGRASP.objectives);
  ctrl->addObjectives(PREGRASP.objectives);

  while(!isFeasible(GRASP, ctrl->komo.pathConfig, true, 1e-2)) step();
  if(verbose){ cout <<"GRASP is feasible" <<endl;  rai::wait();  tic.reset(tau); }

  ctrl->delObjectives(PREGRASP.objectives);
  ctrl->addObjectives(GRASP.objectives);
  S->closeGripper(gripper, .05, .2);

  while(!S->getGripperIsGrasping(gripper)) step();
  if(verbose){ cout <<"GRIPPER is grasping something" <<endl;  rai::wait();  tic.reset(tau); }

  ctrl->delObjectives(GRASP.objectives);
//  ctrl->addObjectives(LIFT.objectives);
  ctrl->addObjectives(BOX.objectives);

  while(/*!isFeasible(LIFT, ctrl->komo.configurations, false, 1e-2)
        ||*/ !isFeasible(BOX, ctrl->komo.pathConfig, false, 1e-2)) step();
  if(verbose){ cout <<"LIFT is done" <<endl;  rai::wait();  tic.reset(tau); }

//  ctrl->delObjectives(LIFT.objectives);
  ctrl->delObjectives(BOX.objectives);
  S->openGripper(gripper, .07, 3.);
  while(!S->getGripperIsOpen(gripper)) step();
  if(verbose){ cout <<"GRIPPER is open" <<endl;  rai::wait(); }
}

//===========================================================================

struct CtrlSetPlus : CtrlSet {
  //just a CtrlSet plus additional command to open/close gripper
  const char* closeGripper=0;
  const char* openGripper=0;
  const char* needsCloseGripper=0;
  const char* needsOpenGripper=0;

  CtrlSetPlus(){}
  CtrlSetPlus(const CtrlSet& CS) : CtrlSet(CS) {}
};

//===========================================================================

void operator<<(ostream& os, const CtrlSet& s){ s.report(os); }
bool operator==(const CtrlSet& a, const CtrlSet& b){ return (&a==&b); }

void EndlessPicker::pickAndLiftBehavior(const char* gripper, const char* gripperCenter, const char* object){
  tic.reset(tau);
  bool verbose=false;

  //create a set of 5 control modes
  CtrlSetPlus NIL;
  CtrlSetPlus APPROACH = symbol2CtrlSet({"prePreGrasp", gripper, object}, ctrl->komo.world);
  CtrlSetPlus ALIGN = symbol2CtrlSet({"preGrasp", gripper, object}, ctrl->komo.world);
  CtrlSetPlus CLOSE = symbol2CtrlSet({"grasp", gripper, object}, ctrl->komo.world);
  CtrlSetPlus RET = symbol2CtrlSet({"box", gripper, "R_workspace"}, ctrl->komo.world, {.3,.3,.1});

  APPROACH.openGripper=gripper;
  ALIGN.needsOpenGripper=gripper;
  CLOSE.closeGripper=gripper;
  RET.needsCloseGripper=gripper;
  NIL.needsCloseGripper=gripper;

  //put them in a list in reverse order
  rai::Array<CtrlSetPlus*> CS = { &RET, &CLOSE, &ALIGN, &APPROACH, &NIL };

  CtrlSetPlus *currentMode = CS.last();

  for(;;){

    //stop condition - wouldn't usually exist!
    CtrlSetPlus* cs = CS.first();
    bool fea = isFeasible(*cs, ctrl->komo.pathConfig, false, 1e-2);
    if(cs->needsCloseGripper) fea = fea && S->getGripperIsGrasping(cs->needsCloseGripper);
    if(fea){
      ctrl->delObjectives(currentMode->objectives);
      S->openGripper(gripper, .07, 3.);
      break;
    }

    //the default logic to pick the current ctrl mode in EACH control step
    for(CtrlSetPlus* mode : CS){ //go forward through the list, pick the first that fulfils conditions
      bool condition = isFeasible(*mode, ctrl->komo.pathConfig, true, 1e-2);
      if(mode->needsCloseGripper) condition = condition && S->getGripperIsGrasping(mode->needsCloseGripper);
      if(mode->needsOpenGripper) condition = condition && S->getGripperIsOpen(mode->needsOpenGripper);
      if(condition){
        if(currentMode!=mode){
          ctrl->delObjectives(currentMode->objectives);
          currentMode = mode;
          ctrl->addObjectives(currentMode->objectives);
          if(mode->closeGripper) S->closeGripper(mode->closeGripper, .05, .2);
          if(mode->openGripper)  S->openGripper(mode->openGripper, .07, 3.);
        }
        break;
      }
    }

    step();
  }
}

