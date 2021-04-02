/*
 * INO_t41_ToFBeacon_fusion.cpp
 *
 *  Created on: Jan 25, 2021
 *      Author: cho (PM-ROBOTIX)
 */

#include "INO_ToF_DetectionBeacon.h"

int ii = 0;


void setup() {
    delay(3000);
    Serial.begin(921600); //fast serial
    Wire.begin();
    Wire.setClock(1000000); // use fast mode I2C
    Wire1.begin();
    Wire1.setClock(1000000); // use fast mode I2C

    // Just to know which program is running on my Arduino
    Serial.println(F("START " __FILE__));

    // initialize the digital pin as an output for LEDS
    pinMode(LED_BUILTIN, OUTPUT); // Green
    pinMode(LED_BUILTIN + 1, OUTPUT); //Red
    pinMode(LED_BUILTIN + 2, OUTPUT); //optional


    ledPanels_setup();

    tof_setup();
}


void loop() {
    ii++;
    if (ii % 2) {
        digitalWrite(LED_BUILTIN, HIGH);
        digitalWrite(LED_BUILTIN + 1, HIGH);
    }
    else {
        digitalWrite(LED_BUILTIN, LOW);
        digitalWrite(LED_BUILTIN + 1, LOW);
    }

    //Write ToF sensors in Serial
    tof_loop(1);

    ledPanels_loop();


}
