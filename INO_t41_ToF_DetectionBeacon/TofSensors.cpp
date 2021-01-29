/*
 * TofSensors.cpp
 *
 *  Created on: Jan 25, 2021
 *      Author: cho (PM-ROBOTIX)
 */

#include "TofSensors.h"


//front:Violet41-Jaune40-Orange39-Rouge38-Marron37-Noir36-Blanc35-Gris34-violet33
//Back:Violet05-Jaune06-Orange26-Rouge27-Marron28-Noir29-Blanc30-Gris31-violet32
int shutd_pin[18] = { 41, 40, 39, 38, 37, 36, 35, 34, 33, 05, 06, 26, 27, 28, 29, 30, 31, 32 };

//centre d'après le tableau (inversé de gauche vers la droite à cause du sens des VL)
int center[16] = { 251, 243, 235, 227, 219, 211, 203, 195, 187, 179, 171, 163, 155, 147, 139, 131 };

char buffer[20];
SFEVL53L1X vl[NumOfSensors];
VL53L1X_Result_t res[NumOfZonesPerSensor * NumOfSensors];

//TODO remplacer tout cela par plusieurs copie de res sur les 5 dernières fois
int16_t filteredResult[NumOfZonesPerSensor * NumOfSensors];
int16_t distance_t[NumOfZonesPerSensor * NumOfSensors];
int16_t status_t[NumOfZonesPerSensor * NumOfSensors];
int16_t NumSPADs_t[NumOfZonesPerSensor * NumOfSensors];
int16_t SigPerSPAD_t[NumOfZonesPerSensor * NumOfSensors];
int16_t Ambient_t[NumOfZonesPerSensor * NumOfSensors];

int16_t OffsetCal[NumOfZonesPerSensor * 9] = { 60, 110, 134, 95, 60, 110, 134, 95, 60, 110, 134, 95, 60, 110, 134, 95, 60, 110, 134, 95, 60, 110, 134,
        95, 60, 110, 134, 95, 60, 110, 134, 95, 60, 110, 134, 95 };
int16_t OffsetCalxTalk[NumOfZonesPerSensor * 9] = { 0, 85, 31, 117, 0, 85, 31, 117, 0, 85, 31, 117, 0, 85, 31, 117, 0, 85, 31, 117, 0, 85, 31, 117, 0,
        85, 31, 117, 0, 85, 31, 117, 0, 85, 31, 117 };

volatile int shared_endloop1 = 0;
volatile int shared_endloop2 = 0;
elapsedMicros elapsedT_us = 0;

void tof_loop(int debug) {
    long t_start = elapsedT_us;
    //TODO si l'un VL est deconnecté
    //TODO si Off/ON alors reinit
    //TODO si perte de signal sur un des cables i2C

    for (int n = 0; n < NumOfSensors; n++) {
        for (int z = 0; z < NumOfZonesPerSensor; z++) {
            Serial.print(filteredResult[(NumOfZonesPerSensor * n) + z]);
            Serial.print(",");
        }
    }
    Serial.println();

    long t_writeserial = elapsedT_us;

    //synchronisation des 2 threads
    while (!shared_endloop1 || !shared_endloop2) {
        threads.yield();
    }
    long t_waitthreads = elapsedT_us;

    if (debug)
    {


        for (int n = 0; n < NumOfSensors; n++) {
            memset(buffer, 0, strlen(buffer));
            for (int z = 0; z < NumOfZonesPerSensor; z++) {
                sprintf(buffer, "%04d ", distance_t[(NumOfZonesPerSensor * n) + z]);
                Serial.print(buffer);
            }
        }
        Serial.println();

        //print debug time
        Serial.println(" t_start=" + String(t_start) +" t_writeserial=" + String(t_writeserial- t_start) + " t_waitthreads=" + String(t_waitthreads - t_writeserial));
    }
    elapsedT_us = 0;
    shared_endloop1 = 0;
    shared_endloop2 = 0;

    threads.yield();

}

int scani2c(TwoWire w) {
    Serial.println("I2C scanner. Scanning ...");
    int count = 0;
    for (byte i = 1; i < 120; i++) {
        w.beginTransmission(i);
        if (w.endTransmission() == 0) {
            //Serial.print("Found address: ");
            Serial.print(i, DEC);
            Serial.print(" (0x");
            Serial.print(i, HEX);
            Serial.print(") ; ");
            count++;
            delay(1);
        }
    }
    Serial.println("Done.");
    Serial.print("Found ");
    Serial.print(count, DEC);
    Serial.println(" device(s).");
    return count;
}

void loopvl1() {

    while (1) {
        while (shared_endloop1) {
            threads.yield();
        }

        for (int z = 0; z < NumOfZonesPerSensor; z++) {

            for (int n = 0; n < NumOfSensors / 2; n++) { //NumOfSensors
                vl[n].setROI(WidthOfSPADsPerZone, 8, center[NumOfSPADsShiftPerZone * z + NumOfSPADsToStartZone]);
                vl[n].startRanging();

            }

            for (int n = 0; n < NumOfSensors / 2; n++) { //NumOfSensors
                while (!vl[n].checkForDataReady()) {
                    //threads.delay(1); //195ms en tout
                    //delay(1); //180ms en tout
                    threads.yield();
                }

                //vl[n].setOffset(OffsetCal[(NumOfZonesPerSensor * n) + z]);
                //vl[n].setXTalk(OffsetCalxTalk[(NumOfZonesPerSensor * n) + z]);
                vl[n].getResult(&res[(NumOfZonesPerSensor * n) + z]);
                vl[n].clearInterrupt();
                vl[n].stopRanging();

                if ((res[(NumOfZonesPerSensor * n) + z].NumSPADs < 5
                        && (res[(NumOfZonesPerSensor * n) + z].Status == RangeValid || res[(NumOfZonesPerSensor * n) + z].Status == PhaseOutOfLimit)
                        && res[(NumOfZonesPerSensor * n) + z].SigPerSPAD > 600 && res[(NumOfZonesPerSensor * n) + z].Ambient < 1100)) //500 25000 si on se fait eblouir
                {

                    filteredResult[(NumOfZonesPerSensor * n) + (3 - z)] = res[(NumOfZonesPerSensor * n) + z].Distance;
                }
                else {
                    filteredResult[(NumOfZonesPerSensor * n) + (3 - z)] = -1;
                }

                //save other data
                distance_t[(NumOfZonesPerSensor * n) + (3 - z)] = res[(NumOfZonesPerSensor * n) + z].Distance;
                status_t[(NumOfZonesPerSensor * n) + (3 - z)] = res[(NumOfZonesPerSensor * n) + z].Status;
                NumSPADs_t[(NumOfZonesPerSensor * n) + (3 - z)] = res[(NumOfZonesPerSensor * n) + z].NumSPADs;
                SigPerSPAD_t[(NumOfZonesPerSensor * n) + (3 - z)] = res[(NumOfZonesPerSensor * n) + z].SigPerSPAD;
                Ambient_t[(NumOfZonesPerSensor * n) + (3 - z)] = res[(NumOfZonesPerSensor * n) + z].Ambient;
            }
        }

        shared_endloop1 = 1;
        threads.yield();
    }
}

void loopvl2() {

    while (1) {
        while (shared_endloop2) {
            threads.yield();
        }

        for (int z = 0; z < NumOfZonesPerSensor; z++) {

            for (int n = (NumOfSensors / 2); n < NumOfSensors; n++) { //NumOfSensors
                vl[n].setROI(WidthOfSPADsPerZone, 8, center[NumOfSPADsShiftPerZone * z + NumOfSPADsToStartZone]);
                vl[n].startRanging();

            }

            for (int n = (NumOfSensors / 2); n < NumOfSensors; n++) { //NumOfSensors
                while (!vl[n].checkForDataReady()) {
                    //threads.delay(1); //195ms en tout
                    //delay(1); //180ms en tout
                    threads.yield();
                }

                //vl[n].setOffset(OffsetCal[(NumOfZonesPerSensor * n) + z]);
                //vl[n].setXTalk(OffsetCalxTalk[(NumOfZonesPerSensor * n) + z]);
                vl[n].getResult(&res[(NumOfZonesPerSensor * n) + z]);
                vl[n].clearInterrupt();
                vl[n].stopRanging();

                if ((res[(NumOfZonesPerSensor * n) + z].NumSPADs < 5
                        && (res[(NumOfZonesPerSensor * n) + z].Status == RangeValid || res[(NumOfZonesPerSensor * n) + z].Status == PhaseOutOfLimit)
                        && res[(NumOfZonesPerSensor * n) + z].SigPerSPAD > 600 && res[(NumOfZonesPerSensor * n) + z].Ambient < 1100)) //500 25000 si on se fait eblouir
                {

                    filteredResult[(NumOfZonesPerSensor * n) + (3 - z)] = res[(NumOfZonesPerSensor * n) + z].Distance;
                }
                else {
                    filteredResult[(NumOfZonesPerSensor * n) + (3 - z)] = -1;
                }

                //save other data
                distance_t[(NumOfZonesPerSensor * n) + (3 - z)] = res[(NumOfZonesPerSensor * n) + z].Distance;
                status_t[(NumOfZonesPerSensor * n) + (3 - z)] = res[(NumOfZonesPerSensor * n) + z].Status;
                NumSPADs_t[(NumOfZonesPerSensor * n) + (3 - z)] = res[(NumOfZonesPerSensor * n) + z].NumSPADs;
                SigPerSPAD_t[(NumOfZonesPerSensor * n) + (3 - z)] = res[(NumOfZonesPerSensor * n) + z].SigPerSPAD;
                Ambient_t[(NumOfZonesPerSensor * n) + (3 - z)] = res[(NumOfZonesPerSensor * n) + z].Ambient;

            }
        }

        shared_endloop2 = 1;
        threads.yield();
    }
}


void tof_setup() {

    for (int i = 0; i < NumOfSensors / 2; i++) {
        vl[i] = SFEVL53L1X(Wire, shutd_pin[i], -1);
    }
    for (int i = NumOfSensors / 2; i < NumOfSensors; i++) {
        vl[i] = SFEVL53L1X(Wire1, shutd_pin[i], -1);
    }

    for (int i = 0; i < NumOfSensors; i++) {
        pinMode(shutd_pin[i], OUTPUT);
    }
    //deactivate all
    for (int i = 0; i < NumOfSensors; i++) {
        digitalWrite(shutd_pin[i], LOW);
    }

    Serial.println("I2C Change address ...");

    for (int i = 0; i < NumOfSensors; i++) {
        pinMode(shutd_pin[i], INPUT);
        delay(10);
        if (vl[i].begin() == 0) {
            vl[i].setI2CAddress((uint8_t) ((0x15 + i) << 1));  //(DEC=21 22 23 ...) //address on 7bits<<1
            digitalWrite(shutd_pin[i], HIGH);
        }
        else {
            Serial.println("ERROR vl[" + String(i) + "] OFFLINE! ");
            //while (1);
        }
    }

    int front = scani2c(Wire);

    //Error missing VL
    if (front != 9) {
        int iii = 0;
        while (1) {
            iii++;
            if (iii % 2) {
                //digitalWrite(LED_BUILTIN, HIGH);
                digitalWrite(LED_BUILTIN + 1, HIGH);
            }
            else {
                //digitalWrite(LED_BUILTIN, LOW);
                digitalWrite(LED_BUILTIN + 1, LOW);
            }
            delay(200);
        }
    }
    int back = scani2c(Wire1);
    if (back != 9) {
        int iii = 0;
        while (1) {
            iii++;
            if (iii % 2) {
                //digitalWrite(LED_BUILTIN, HIGH);
                digitalWrite(LED_BUILTIN + 1, HIGH);
            }
            else {
                //digitalWrite(LED_BUILTIN, LOW);
                digitalWrite(LED_BUILTIN + 1, LOW);
            }
            delay(400);
        }
    }
    Serial.println("Configuration de chaque VL... starting");
    //configuration de chaque VL
    for (int n = 0; n < NumOfSensors; n++) {
        int addr = vl[n].getI2CAddress() >> 1;
        Serial.print(" 0x");
        Serial.print(addr, HEX);

        uint16_t sensorId = vl[n].getSensorID();
        Serial.print(" sensorId=");
        Serial.print(sensorId, HEX);

        int available = vl[n].checkID();
        Serial.print(" up=");
        Serial.println(available);

        vl[n].setDistanceModeShort();
        //vl[n].setDistanceModeLong();
        vl[n].setTimingBudgetInMs(50);
        vl[n].setIntermeasurementPeriod(51); // measure periodically. Intermeasurement period must be >/= timing budget.
    }

    threads.addThread(loopvl1);
    threads.addThread(loopvl2);
}

