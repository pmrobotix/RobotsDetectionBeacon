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

#define TotalWidthOfSPADS               16
#define WidthOfSPADsPerZone             4
#define NumOfSPADsShiftPerZone          4 //decalage de zone
#define NumOfSPADsToStartZone           1 //SENS normal:declage de 2; sens inversé du tableau centre :decalage de 1
#define NumOfZonesPerSensor             (((TotalWidthOfSPADS - WidthOfSPADsPerZone) / NumOfSPADsShiftPerZone) + 1)
#define NumOfSensors                    18

int scani2c(TwoWire w);
void tof_setup();
void tof_loop();
void loopvl1();
void loopvl2();

#endif /* TOFSENSORS_H_ */
