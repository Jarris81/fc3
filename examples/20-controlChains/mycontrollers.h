#pragma once

#include <Control/CtrlSet.h>

//returns a set of control objectives for each string symbol
CtrlSet symbol2CtrlSet(const StringA& symbols, const rai::Configuration& C, const arr& params=arr());
