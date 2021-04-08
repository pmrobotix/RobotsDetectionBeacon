/*
 * INO_t41_ToFBeacon_fusion.cpp
 *
 *  Created on: Jan 25, 2021
 *      Author: cho (PM-ROBOTIX)
 */

#include "INO_ToF_DetectionBeacon.h"



//Variables globales
bool wait_TofVLReady = false;

int16_t filteredResult[NumOfZonesPerSensor * NumOfSensors];
int16_t distance_t[NumOfZonesPerSensor * NumOfSensors];
int16_t status_t[NumOfZonesPerSensor * NumOfSensors];
bool connected_t[NumOfZonesPerSensor * NumOfSensors];
int16_t NumSPADs_t[NumOfZonesPerSensor * NumOfSensors];
int16_t SigPerSPAD_t[NumOfZonesPerSensor * NumOfSensors];
int16_t Ambient_t[NumOfZonesPerSensor * NumOfSensors];

int16_t filteredResult_coll[NumOfCollisionSensors + NumOfCollisionSensors];
int16_t distance_coll[NumOfCollisionSensors + NumOfCollisionSensors];
int16_t status_coll[NumOfCollisionSensors + NumOfCollisionSensors];
bool connected_coll[NumOfCollisionSensors + NumOfCollisionSensors];
int16_t NumSPADs_coll[NumOfZonesPerSensor * NumOfSensors];
int16_t SigPerSPAD_coll[NumOfZonesPerSensor * NumOfSensors];
int16_t Ambient_coll[NumOfZonesPerSensor * NumOfSensors];

int nb_active_filtered_sensors;
int videoMode;
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


    videoMode = 0;

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
    tof_loop(false);

    ledPanels_loop(false);
}
