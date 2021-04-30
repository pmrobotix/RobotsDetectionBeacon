/*
 * TofSensors.cpp
 *
 *  Created on: Jan 25, 2021
 *      Author: cho (PM-ROBOTIX)
 */

#include "INO_ToF_DetectionBeacon.h"
#include "TofSensors.h"
#include <i2c_register_slave.h>
//
//old- front:Violet41-Jaune40-Orange39-Rouge38-Marron37-Noir36-Blanc35-Gris34-violet33
//old- Back:Violet05-Jaune06-Orange26-Rouge27-Marron28-Noir29-Blanc30-Gris31-violet32
//old- int shutd_pin[18] = { 41, 40, 39, 38, 37, 36, 35, 34, 33,  05, 06, 26, 27, 28, 29, 30, 31, 32 };

//Nouvelle config avec detecteurs
// front[9] back[9]
//front[9]:Violet41-Jaune40-Orange39-Rouge38-Marron37-Noir36-Blanc35-Gris34-violet33
//Back[9]:Violet20-Jaune21-Orange22-Rouge23-Marron4-Noir3-Blanc2-Gris1-violet0
int shutd_pin[NumOfSensors] = { 20, 21, 22, 23, 4, 3, 2, 1, 0, 5, 6, 26, 27, 28, 29, 30, 31, 32 };

//front[4]= BAS HAUT BAS HAUT  back[4]
int shutd_pin_collision[NumOfCollisionSensors + NumOfCollisionSensors] = { 40, 39, 38, 37, 36, 35, 34, 33 };

//centre des SPADs d'après le tableau (inversé de gauche vers la droite à cause du sens des VL)
int center[16] = { 251, 243, 235, 227, 219, 211, 203, 195, 187, 179, 171, 163, 155, 147, 139, 131 };

char buffer[24];

SFEVL53L1X vl[NumOfSensors];
VL53L1X_Result_t res[NumOfZonesPerSensor * NumOfSensors];

SFEVL53L1X vl_collision[NumOfCollisionSensors + NumOfCollisionSensors];
VL53L1X_Result_t res_collision[NumOfCollisionSensors + NumOfCollisionSensors];
extern bool connected_coll[NumOfCollisionSensors + NumOfCollisionSensors];
extern int16_t filteredResult_coll[NumOfCollisionSensors + NumOfCollisionSensors];
extern int16_t distance_coll[NumOfCollisionSensors + NumOfCollisionSensors];
extern int16_t status_coll[NumOfCollisionSensors + NumOfCollisionSensors];
extern int16_t NumSPADs_coll[NumOfZonesPerSensor * NumOfSensors];
extern int16_t SigPerSPAD_coll[NumOfZonesPerSensor * NumOfSensors];
extern int16_t Ambient_coll[NumOfZonesPerSensor * NumOfSensors];

//TODO remplacer tout cela par plusieurs copie de res sur les 5 dernières fois?

extern bool wait_TofVLReady;
extern int nb_active_filtered_sensors;
extern int videoMode;

extern bool connected_t[NumOfZonesPerSensor * NumOfSensors];
extern int16_t filteredResult[NumOfZonesPerSensor * NumOfSensors];
extern int16_t distance_t[NumOfZonesPerSensor * NumOfSensors];
extern int16_t status_t[NumOfZonesPerSensor * NumOfSensors];
extern int16_t NumSPADs_t[NumOfZonesPerSensor * NumOfSensors];
extern int16_t SigPerSPAD_t[NumOfZonesPerSensor * NumOfSensors];
extern int16_t Ambient_t[NumOfZonesPerSensor * NumOfSensors];


int16_t OffsetCal[NumOfZonesPerSensor * 9] = { 60, 110, 134, 95, 60, 110, 134, 95, 60, 110, 134, 95, 60, 110, 134, 95, 60, 110, 134, 95, 60, 110, 134,
        95, 60, 110, 134, 95, 60, 110, 134, 95, 60, 110, 134, 95 };
int16_t OffsetCalxTalk[NumOfZonesPerSensor * 9] = { 0, 85, 31, 117, 0, 85, 31, 117, 0, 85, 31, 117, 0, 85, 31, 117, 0, 85, 31, 117, 0, 85, 31, 117, 0,
        85, 31, 117, 0, 85, 31, 117, 0, 85, 31, 117 };

volatile int shared_endloop1 = 0;
volatile int shared_endloop2 = 0;

elapsedMicros elapsedT_us = 0;



void tof_setup() {

    //Front vl on i2c 18 SDA / 19 SCL
    for (int i = 0; i < NumOfSensors / 2; i++) {
        vl[i] = SFEVL53L1X(Wire, shutd_pin[i], -1);
    }
    //Back vl on i2c  17 SDA1 / 16 SCL1
    for (int i = NumOfSensors / 2; i < NumOfSensors; i++) {
        vl[i] = SFEVL53L1X(Wire1, shutd_pin[i], -1);
    }

    //config collision Front vl on i2c 18 SDA / 19 SCL
    for (int i = 0; i < (NumOfCollisionSensors); i++) {
        vl_collision[i] = SFEVL53L1X(Wire, shutd_pin_collision[i], -1);
    }
    //config collision Back vl on i2c  17 SDA1 / 16 SCL1
    for (int i = NumOfCollisionSensors; i < (NumOfCollisionSensors + NumOfCollisionSensors); i++) {
        vl_collision[i] = SFEVL53L1X(Wire1, shutd_pin_collision[i], -1);
    }

    //Config all shutpin
    for (int i = 0; i < NumOfSensors; i++) {
        pinMode(shutd_pin[i], OUTPUT);
    }
    for (int i = 0; i < NumOfCollisionSensors + NumOfCollisionSensors; i++) {
        pinMode(shutd_pin_collision[i], OUTPUT);
    }

    //deactivate all
    for (int i = 0; i < NumOfSensors; i++) {
        digitalWrite(shutd_pin[i], LOW);
    }
    for (int i = 0; i < NumOfCollisionSensors + NumOfCollisionSensors; i++) {
        digitalWrite(shutd_pin_collision[i], LOW);
    }

    Serial.println("vl I2C Change address ...");
//    I2C scanner. Scanning ...
//    21 (0x15) ; 22 (0x16) ; 23 (0x17) ; 24 (0x18) ; 25 (0x19) ; 26 (0x1A) ; 27 (0x1B) ; 28 (0x1C) ; 29 (0x1D) ; Done.
//    Found 9 device(s).
//    I2C scanner. Scanning ...
//    30 (0x1E) ; 31 (0x1F) ; 32 (0x20) ; 33 (0x21) ; 34 (0x22) ; 35 (0x23) ; 38 (0x26) ; 43 (0x2B) ; 44 (0x2C) ; 45 (0x2D) ; 46 (0x2E) ; Done.
//    Found 11 device(s).
//    Config VL (FRONT):0=0x15 1=0x16 2=0x17 3=0x18 4=0x19 5=0x1A 6=0x1B 7=0x1C 8=0x1D
//    Config VL  (BACK):9=0x1E 10=0x1F 11=0x20 12=0x21 13=0x22 14=0x23 15=[0x29 ERROR] 16=[0x29 ERROR] 17=0x26
//    Config VL  (COLL):0=[0x29 ERROR] 1=[0x29 ERROR] 2=[0x29 ERROR] 3=[0x29 ERROR] 4=0x2B 5=0x2C 6=0x2D 7=0x2E

    for (int i = 0; i < NumOfSensors; i++) {
        pinMode(shutd_pin[i], INPUT);
        delay(10);
        if (vl[i].begin() == 0) {
            vl[i].setI2CAddress((uint8_t) ((0x15 + i) << 1));  //(DEC=21 22 23 ...) //address on 7bits<<1
            digitalWrite(shutd_pin[i], HIGH);
            for (int z = 0; z < NumOfZonesPerSensor; z++) {
                connected_t[(NumOfZonesPerSensor * i) + z] = true;
            }
        }
        else {
            Serial.print("ERROR vl[" + String(i) + "] OFFLINE! ");
            Serial.print(" 0x");
            Serial.println((0x15 + i), HEX);
            for (int z = 0; z < NumOfZonesPerSensor; z++) {
                connected_t[(NumOfZonesPerSensor * i) + z] = false;
            }
            //while (1);
        }
    }

    Serial.println("vl collision I2C Change address ...");

    for (int i = 0; i < (NumOfCollisionSensors + NumOfCollisionSensors); i++) {
        pinMode(shutd_pin_collision[i], INPUT);
        delay(10);
        if (vl_collision[i].begin() == 0) {
            vl_collision[i].setI2CAddress((uint8_t) ((0x30 + i) << 1));  //(HEX=27 28 29 2A ...) //address on 7bits<<1
            digitalWrite(shutd_pin_collision[i], HIGH);

            connected_coll[i] = true;

        }
        else {
            Serial.print("ERROR vl_collision[" + String(i) + "] OFFLINE! ");
            Serial.print(" 0x");
            Serial.println((0x30 + i), HEX);

            connected_coll[i] = false;

            //while (1);
        }
    }



////Error missing VL
//        int iii = 0;
//        while (1) {
//            iii++;
//            if (iii % 2) {
//                //digitalWrite(LED_BUILTIN, HIGH);
//                digitalWrite(LED_BUILTIN + 1, HIGH);
//            }
//            else {
//                //digitalWrite(LED_BUILTIN, LOW);
//                digitalWrite(LED_BUILTIN + 1, LOW);
//            }
//            delay(200);
//        }

    Serial.print("Config VL (FRONT):");
    //configuration de chaque VL
    for (int n = 0; n < NumOfSensors; n++) {
        if (connected_t[n]) {
            int available = vl[n].checkID();
            if (available) {
                //connected_t[n] = true;
                int addr = vl[n].getI2CAddress() >> 1;
                Serial.print(String(n) + "=0x");
                Serial.print(addr, HEX);
                Serial.print(" ");
                //uint16_t sensorId = vl[n].getSensorID();

                vl[n].setDistanceModeShort(); //Short to have relevant figures
                vl[n].setTimingBudgetInMs(50); //50
                vl[n].setIntermeasurementPeriod(51); //51 measure periodically. Intermeasurement period must be >/= timing budget.
                if (n == 8) {
                    Serial.println();
                    Serial.print("Config VL  (BACK):");
                }
            }
            else {
                Serial.print(String(n) + "=[0x");
                Serial.print(vl[n].getI2CAddress() >> 1, HEX);
                Serial.print(" ERROR");
                Serial.print("] ");

                for (int z = 0; z < NumOfZonesPerSensor; z++) {
                    connected_t[(NumOfZonesPerSensor * n) + z] = false;
                }
                status_t[n] = 8;
            }
        }
    }
    Serial.println();

    //Configuration de chaque VL COLLISION
    Serial.print("Config VL  (COLL):");
    for (int n = 0; n < (NumOfCollisionSensors + NumOfCollisionSensors); n++) {
        if (connected_coll[n]) {
            int available = vl_collision[n].checkID();
            if (available) {
                //connected_coll[n] = true;
                int addr = vl_collision[n].getI2CAddress() >> 1;
                Serial.print(String(n) + "=0x");
                Serial.print(addr, HEX);
                Serial.print(" ");
                //uint16_t sensorId = vl_collision[n].getSensorID();

                vl_collision[n].setDistanceModeShort(); //Short to have relevant figures
                vl_collision[n].setTimingBudgetInMs(50); //50
                vl_collision[n].setIntermeasurementPeriod(51); //51 measure periodically. Intermeasurement period must be >/= timing budget.
                if (n == 8) Serial.println();
            }
            else {
                Serial.print(String(n) + "= ");
                //Serial.print(vl_collision[n].getI2CAddress() >> 1, HEX);
                Serial.print(" ERROR");
                Serial.print("] ");

                connected_coll[n] = false;

                status_coll[n] = 8;
            }
        }
    }
    Serial.println();

    threads.delay(1000);
    wait_TofVLReady = true;

    threads.addThread(loopvl1);
    threads.addThread(loopvl2);
}




//Attention pas de delay dans cette fonction pour ne pas ralentir la main loop
void tof_loop(Registers &registers, int debug) {
    long t_start = elapsedT_us;



    //TODO si l'un VL est deconnecté
    //TODO si Off/ON alors reinit
    //TODO si perte de signal sur un des cables i2C
    //TODO passage de la balise une partie en debut et à la fin
    //TODO detecter 3 balises ?



    //Write ToF sensors in Serial
//    for (int n = 0; n < (NumOfCollisionSensors + NumOfCollisionSensors); n++) {
//
//        Serial.print(filteredResult_coll[n]);
//        Serial.print(",");
//    }
//    Serial.println();
    nb_active_filtered_sensors = 0;
    for (int n = 0; n < NumOfSensors; n++) {
        for (int z = 0; z < NumOfZonesPerSensor; z++) {
            int val = filteredResult[(NumOfZonesPerSensor * n) + z];
            if (val > 0)
                nb_active_filtered_sensors++;
            Serial.print(val);
            Serial.print(",");
        }
    }
    Serial.println();
    Serial.println();

    long t_writeserial = elapsedT_us;

    //change mode
    if (nb_active_filtered_sensors > NumOfSensorsForVideoMode)
    {
        videoMode = 1;
    }


    //print info debug
    if (debug) {
        for (int n = 0; n < NumOfSensors; n++) {
            memset(buffer, 0, strlen(buffer));
            for (int z = 0; z < NumOfZonesPerSensor; z++) {
                sprintf(buffer, "%04d ", distance_t[(NumOfZonesPerSensor * n) + z]);
                Serial.print(buffer);
            }
        }
        Serial.println();

        for (int n = 0; n < NumOfSensors; n++) {
            memset(buffer, 0, strlen(buffer));
            for (int z = 0; z < NumOfZonesPerSensor; z++) {
                sprintf(buffer, "%01d  ", status_t[(NumOfZonesPerSensor * n) + z]);
                Serial.print(buffer);
            }
        }
        Serial.println();
        for (int n = 0; n < NumOfSensors; n++) {
            memset(buffer, 0, strlen(buffer));
            for (int z = 0; z < NumOfZonesPerSensor; z++) {
                sprintf(buffer, "%01d  ", NumSPADs_t[(NumOfZonesPerSensor * n) + z]);
                Serial.print(buffer);
            }
        }
        Serial.println();
        for (int n = 0; n < NumOfSensors; n++) {
            memset(buffer, 0, strlen(buffer));
            for (int z = 0; z < NumOfZonesPerSensor; z++) {
                sprintf(buffer, "%01d  ", SigPerSPAD_t[(NumOfZonesPerSensor * n) + z]);
                Serial.print(buffer);
            }
        }
        Serial.println();
        for (int n = 0; n < NumOfSensors; n++) {
            memset(buffer, 0, strlen(buffer));
            for (int z = 0; z < NumOfZonesPerSensor; z++) {
                sprintf(buffer, "%01d  ", Ambient_t[(NumOfZonesPerSensor * n) + z]);
                Serial.print(buffer);
            }
        }
        Serial.println();

//        for (int n = 0; n < NumOfSensors; n++) {
//            memset(buffer, 0, strlen(buffer));
//            for (int z = 0; z < NumOfZonesPerSensor; z++) {
//                sprintf(buffer, "%01d  ", connected_t[(NumOfZonesPerSensor * n) + z]);
//                Serial.print(buffer);
//            }
//        }
//        Serial.println();


    }
    long t_printDebug = elapsedT_us;


    //synchronisation des 2 threads de VL
    while (!shared_endloop1 || !shared_endloop2) {
        threads.yield();
    }
    long t_waitthreads = elapsedT_us;

    //print debug time
    Serial.println(
            " t_start=" + String(t_start)
            + " t_writeserial=" + String(t_writeserial - t_start)
            + " t_printDebug=" + String(t_printDebug - t_writeserial)
            + " t_waitthreads=" + String(t_waitthreads - t_printDebug));
    Serial.println();


    elapsedT_us = 0;
    shared_endloop1 = 0; //lancement de la mise à jour des données loop1
    shared_endloop2 = 0; //lancement de la mise à jour des données loop2

    threads.yield();
}


/**Table of Optical Centers**
 *
 * 128,136,144,152,160,168,176,184,  192,200,208,216,224,232,240,248
 * 129,137,145,153,161,169,177,185,  193,201,209,217,225,233,241,249
 * 130,138,146,154,162,170,178,186,  194,202,210,218,226,234,242,250
 * 131,139,147,155,163,171,179,187,  195,203,211,219,227,235,243,251
 * 132,140,148,156,164,172,180,188,  196,204,212,220,228,236,244,252
 * 133,141,149,157,165,173,181,189,  197,205,213,221,229,237,245,253
 * 134,142,150,158,166,174,182,190,  198,206,214,222,230,238,246,254
 * 135,143,151,159,167,175,183,191,  199,207,215,223,231,239,247,255

 * 127,119,111,103, 95, 87, 79, 71,  63, 55, 47, 39, 31, 23, 15, 7
 * 126,118,110,102, 94, 86, 78, 70,  62, 54, 46, 38, 30, 22, 14, 6
 * 125,117,109,101, 93, 85, 77, 69,  61, 53, 45, 37, 29, 21, 13, 5
 * 124,116,108,100, 92, 84, 76, 68,  60, 52, 44, 36, 28, 20, 12, 4
 * 123,115,107, 99, 91, 83, 75, 67,  59, 51, 43, 35, 27, 19, 11, 3
 * 122,114,106, 98, 90, 82, 74, 66,  58, 50, 42, 34, 26, 18, 10, 2
 * 121,113,105, 97, 89, 81, 73, 65,  57, 49, 41, 33, 25, 17, 9, 1
 * 120,112,104, 96, 88, 80, 72, 64,  56, 48, 40, 32, 24, 16, 8, 0 Pin 1
 *
 * To set the center, set the pad that is to the right and above the exact center of the region you'd like to measure as your opticalCenter*/

//Front loop
void loopvl1() {

    while (1) {
        while (shared_endloop1) {
            threads.yield();
        }

        for (int z = 0; z < NumOfZonesPerSensor; z++) {

            if (z == 0) //Gestion collision
                    {
                for (int c = 0; c < NumOfCollisionSensors; c++) {
                    if (connected_coll[c]) {
                        vl_collision[c].setROI(16, 16, 199); //full matrix
                        vl_collision[c].startRanging();
                    }
                }
            }
            for (int n = 0; n < NumOfSensors / 2; n++) { //NumOfSensors
                if (!vl[n].checkID()) connected_t[(NumOfZonesPerSensor * n) + z] = false;
                if (connected_t[(NumOfZonesPerSensor * n) + z]) {
                    vl[n].setROI(WidthOfSPADsPerZone, 8, center[NumOfSPADsShiftPerZone * z + NumOfSPADsToStartZone]);
                    vl[n].startRanging();
                }
            }

            if (z == 0) { //Gestion collision
                for (int c = 0; c < NumOfCollisionSensors; c++) {
                    if (connected_coll[c]) {
                        while (!vl_collision[c].checkForDataReady()) {
                            threads.yield();
                        }
                        vl_collision[c].getResult(&res_collision[c]);
                        vl_collision[c].clearInterrupt();
                        vl_collision[c].stopRanging();

                        filteredResult_coll[c] = res_collision[c].Distance;
                        distance_coll[c] = res_collision[c].Distance;
                        status_coll[c] = res_collision[c].Status;
                        NumSPADs_coll[c] = res_collision[c].NumSPADs;
                        SigPerSPAD_coll[c] = res_collision[c].SigPerSPAD;
                        Ambient_coll[c] = res_collision[c].Ambient;
                    }
                }
            }
            for (int n = 0; n < NumOfSensors / 2; n++) { //NumOfSensors
                if (connected_t[(NumOfZonesPerSensor * n) + z]) {
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

                    //(3 - z) c'est pour inverser les capteurs et lire les zones dans l'autre sens
                    if ((res[(NumOfZonesPerSensor * n) + z].NumSPADs < 5
                            && (res[(NumOfZonesPerSensor * n) + z].Status == RangeValid
                                    || res[(NumOfZonesPerSensor * n) + z].Status == PhaseOutOfLimit)
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
                else {
                    filteredResult[(NumOfZonesPerSensor * n) + (3 - z)] = -2;
                    distance_t[(NumOfZonesPerSensor * n) + (3 - z)] = -2;
                    status_t[(NumOfZonesPerSensor * n) + (3 - z)] = -2;
                }
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
            if (z == 0) { //Gestion collision
                for (int c = NumOfCollisionSensors; c < NumOfCollisionSensors + NumOfCollisionSensors; c++) {
                    if (connected_coll[c]) {
                        vl_collision[c].setROI(16, 16, 199);
                        vl_collision[c].startRanging();
                    }
                }
            }
            for (int n = (NumOfSensors / 2); n < NumOfSensors; n++) { //NumOfSensors
                if (!vl[n].checkID()) connected_t[(NumOfZonesPerSensor * n) + z] = false;
                if (connected_t[(NumOfZonesPerSensor * n) + z]) {
                    vl[n].setROI(WidthOfSPADsPerZone, 8, center[NumOfSPADsShiftPerZone * z + NumOfSPADsToStartZone]);
                    vl[n].startRanging();
                }
            }

            if (z == 0) //Gestion collision
                    {
                for (int c = NumOfCollisionSensors; c < NumOfCollisionSensors + NumOfCollisionSensors; c++) {
                    if (connected_coll[c]) {
                        while (!vl_collision[c].checkForDataReady()) {
                            threads.yield();
                        }
                        vl_collision[c].getResult(&res_collision[c]);
                        vl_collision[c].clearInterrupt();
                        vl_collision[c].stopRanging();

                        filteredResult_coll[c] = res_collision[c].Distance;
                        distance_coll[c] = res_collision[c].Distance;
                        status_coll[c] = res_collision[c].Status;
                        NumSPADs_coll[c] = res_collision[c].NumSPADs;
                        SigPerSPAD_coll[c] = res_collision[c].SigPerSPAD;
                        Ambient_coll[c] = res_collision[c].Ambient;
                    }
                }
            }

            for (int n = (NumOfSensors / 2); n < NumOfSensors; n++) { //NumOfSensors
                if (connected_t[(NumOfZonesPerSensor * n) + z]) {
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
                            && (res[(NumOfZonesPerSensor * n) + z].Status == RangeValid
                                    || res[(NumOfZonesPerSensor * n) + z].Status == PhaseOutOfLimit)
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
                else {
                    filteredResult[(NumOfZonesPerSensor * n) + (3 - z)] = -2;
                    distance_t[(NumOfZonesPerSensor * n) + (3 - z)] = -2;
                    status_t[(NumOfZonesPerSensor * n) + (3 - z)] = -2;
                }
            }
        }

        shared_endloop2 = 1;
        threads.yield();
    }
}

