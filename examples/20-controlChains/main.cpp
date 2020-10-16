#include "myspace.h"

//===========================================================================

int main(int argc,char **argv){
  rai::initCmdLine(argc,argv);

  EndlessPicker M;
  M.setupSim(false);
  M.setupC();
  M.setupCtrl();

  for(uint k=0;k<10;k++){
//    M.pickAndLiftScript("R_gripper", "R_gripperCenter", "object");
    M.pickAndLiftBehavior("R_gripper", "R_gripperCenter", "object");
  }

  return 0;
}
