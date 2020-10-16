#include "mycontrollers.h"

#include <Kin/featureSymbols.h>

CtrlSet symbol2CtrlSet(const StringA& symbols, const rai::Configuration& C, const arr& params){
  CtrlSet ctrl;
  if(!symbols.N){
    HALT("I need symbols!"); //at least one symbol

  } else if(symbols(0)=="prePreGrasp") {
    //transient: center object in hand from top (the last number is the MaxCarrot-maxDistance, which makes it transient (=with moving target))
    ctrl.addObjective(symbols2feature(FS_vectorZ, {STRING(symbols(1) <<"Center")}, C, {1e1}, {0., 0., 1.}), OT_sos, .005);
    ctrl.addObjective(symbols2feature(FS_positionRel, {symbols(2), symbols(1)}, C, {1e1}, {.0, 0., -.2}), OT_sos, .002);

  } else if(symbols(0)=="preGrasp") {
    //funnel constraints:
    ctrl.addObjective(symbols2feature(FS_insideBox, {symbols(2), STRING(symbols(1) <<"Pregrasp")}, C, {1e0}), OT_ineq, -1.);
    //transient:
    ctrl.addObjective(symbols2feature(FS_vectorZ, {STRING(symbols(1) <<"Center")}, C, {1e1}, {0., 0., 1.}), OT_sos, .005);
    ctrl.addObjective(symbols2feature(FS_positionDiff, {STRING(symbols(1) <<"Center"), symbols(2)}, C, {1e1}), OT_sos, .002);

  } else if(symbols(0)=="grasp") {
    ctrl.addObjective(symbols2feature(FS_vectorZ, {STRING(symbols(1) <<"Center")}, C, {}, {0., 0., 1.}), OT_eq, -1.);
    ctrl.addObjective(symbols2feature(FS_positionDiff, {STRING(symbols(1) <<"Center"), symbols(2)}, C, {1e1}), OT_eq, -1.);

  } else if(symbols(0)=="lift") {
    ctrl.addObjective(symbols2feature(FS_position, {symbols(1)}, C, arr({1,3}, {0.,0.,-1e1}), arr({0.,0.,.8}) ), OT_ineq, .002);

  } else if(symbols(0)=="box") {
    ctrl.addObjective(symbols2feature(FS_insideBox, {symbols(1), symbols(2)}, C, {1e1}), OT_ineq, .002);

  } else HALT("can't interpret feature symbols: " <<symbols(0));


  return ctrl;
}
