/*
 * TofSensors.h
 *
 *  Created on: Jan 25, 2021
 *      Author: cho (PM-ROBOTIX)
 */

#ifndef TOFSENSORS_H_
#define TOFSENSORS_H_

#include "SparkFun_VL53L1X.h"
#include "TeensyThreads.h"

int scani2c(TwoWire w);
void tof_setup();
void tof_loop(int debug=0);
void loopvl1();
void loopvl2();

#endif /* TOFSENSORS_H_ */
