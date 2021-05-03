/*
 * INO_t41_ToFBeacon_fusion.cpp
 *
 *  Created on: Jan 25, 2021
 *      Author: cho (PM-ROBOTIX)
 */

#include "INO_ToF_DetectionBeacon.h"
#include <i2c_register_slave.h>

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

Settings settings = { 3, 1 };
Registers registers;
I2CRegisterSlave registerSlave = I2CRegisterSlave(Slave2, (uint8_t*) &settings, sizeof(Settings), (uint8_t*) &registers, sizeof(Registers));

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

void on_read_isr(uint8_t reg_num) {
    // Clear the "new data" bit so the master knows it's
    // already read this set of values.
    registers.flags = 0;
}

void setup() {
    delay(3000);
    Serial.begin(921600); //fast serial
    Wire.setClock(1000000); // use fast mode I2C
    Wire.begin();
    Wire1.setClock(1000000); // use fast mode I2C
    Wire1.begin();

    // Start listening on I2C4 with address 0xDD
    registerSlave.listen(0x2D);
    registerSlave.after_read(on_read_isr);

    // Just to know which program is running on my Arduino
    Serial.println(F("START " __FILE__));

    // initialize the digital pin as an output for LEDS
    pinMode(LED_BUILTIN, OUTPUT); // Green
    pinMode(LED_BUILTIN + 1, OUTPUT); //Red
    pinMode(LED_BUILTIN + 2, OUTPUT); //optional

    videoMode = 0;

    ledPanels_setup();

    tof_setup();

    //scan les bus i2c pour verifier les adresses i2c
    scani2c(Wire);
    scani2c(Wire1);

}

int8_t calculPosition(float decalage_deg, Registers &new_values) {

    int filtrage_mm = -1; //attention si on mets plus, par ex 100mm, certaines valeurs sont à zero avec la balise en face et très prets, voir si un calibrage regle le problème  lorsque l'on approche les mains
    //(on filtre en dessous de 150mm ?)
    int max_nb_bots = 7;
    int final_nb_bots = 0;

    int16_t tab[9 * max_nb_bots] = { 0 }; // données des 4 balises z1_p, z1_1=>z1_5
    float pos[3 * max_nb_bots] = { 0.0 };
    int16_t dist[max_nb_bots] = { 0 };
    for (int i = 0; i < 9 * max_nb_bots; i++) {
        tab[i] = -1;
    }
    for (int i = 0; i < 3 * max_nb_bots; i++) {
        pos[i] = -1.0;
    }
    for (int i = 0; i < max_nb_bots; i++) {
        dist[i] = -1;
    }

    int index_tab_bot = 0;

    int p = -1;
    bool end_ = false;

    int chevauchement_arriere_nb = 0;

    //1. déterminer les 4 balises pour log (chevauchement data possible)
    for (int nb_bots = 1; nb_bots <= max_nb_bots; nb_bots++) {

        //on parcours pour trouver une balise (0 à 71)
        for (int n = p + 1; n < (NumOfZonesPerSensor * NumOfSensors); n++) {

            int16_t val = filteredResult[n];
            if (val > filtrage_mm) {
                //on a trouvé un balise
                final_nb_bots = nb_bots;

                if (nb_bots >= 4) {
                    //TODO raise an issue ! Flag
                }

                //Serial.println("beacon found " + String(n) + " " + String(val));
                if (index_tab_bot < 9 * max_nb_bots) {
                    tab[0 + index_tab_bot] = n;
                    tab[1 + index_tab_bot] = 1;
                    tab[2 + index_tab_bot] = val;
                }
                //gestion du chevauchement avec filtrage
                if (n == 0) if (filteredResult[0] > filtrage_mm) {
                    if (filteredResult[(NumOfZonesPerSensor * NumOfSensors) - 1] > filtrage_mm) {
                        chevauchement_arriere_nb = 1;
                        if (filteredResult[(NumOfZonesPerSensor * NumOfSensors) - 2] > filtrage_mm) {
                            chevauchement_arriere_nb = 2;
                            if (filteredResult[(NumOfZonesPerSensor * NumOfSensors) - 3] > filtrage_mm) {
                                chevauchement_arriere_nb = 3;
                                if (filteredResult[(NumOfZonesPerSensor * NumOfSensors) - 4] > filtrage_mm) {
                                    chevauchement_arriere_nb = 4;
                                    if (filteredResult[(NumOfZonesPerSensor * NumOfSensors) - 5] > filtrage_mm) {
                                        chevauchement_arriere_nb = 5;
                                        if (filteredResult[(NumOfZonesPerSensor * NumOfSensors) - 6] > filtrage_mm) {
                                            chevauchement_arriere_nb = 6;
                                        }
                                    }
                                }
                            }
                        }
                    }
                }

                p = n; //enregistrement du dernier
                if (index_tab_bot < 9 * max_nb_bots) {
                    if (n <= (NumOfZonesPerSensor * NumOfSensors) - 2) if (filteredResult[n + 1] >= 0) {
                        tab[3 + index_tab_bot] = filteredResult[n + 1];
                        p = n + 1;
                        tab[1 + index_tab_bot] = 2;
                        if (n <= (NumOfZonesPerSensor * NumOfSensors) - 3) if (filteredResult[n + 2] >= 0) {
                            tab[4 + index_tab_bot] = filteredResult[n + 2];
                            p = n + 2;
                            tab[1 + index_tab_bot] = 3;
                            if (n <= (NumOfZonesPerSensor * NumOfSensors) - 4) if (filteredResult[n + 3] >= 0) {
                                tab[5 + index_tab_bot] = filteredResult[n + 3];
                                p = n + 3;
                                tab[1 + index_tab_bot] = 4;
                                if (n <= (NumOfZonesPerSensor * NumOfSensors) - 5) if (filteredResult[n + 4] >= 0) {
                                    tab[6 + index_tab_bot] = filteredResult[n + 4];
                                    p = n + 4;
                                    tab[1 + index_tab_bot] = 5;
                                    if (n <= (NumOfZonesPerSensor * NumOfSensors) - 6) if (filteredResult[n + 5] >= 0) {
                                        tab[7 + index_tab_bot] = filteredResult[n + 5];
                                        p = n + 5;
                                        tab[1 + index_tab_bot] = 6;
                                        if (n <= (NumOfZonesPerSensor * NumOfSensors) - 7) if (filteredResult[n + 6] >= 0) {
                                            tab[8 + index_tab_bot] = filteredResult[n + 6];
                                            p = n + 6;
                                            tab[1 + index_tab_bot] = 7;
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
                break;
            }
            else {
                if (n >= (NumOfZonesPerSensor * NumOfSensors) - 1) {
                    end_ = true;
                    break;
                }
            }
        }
        if (end_) break;
        if (p == (NumOfZonesPerSensor * NumOfSensors) - 1) break; //cas d'arret avec chevauchement
        index_tab_bot += 9;
    }
    int c = 0;

    //gestion du chevauchement de data, on ne retient que max_balises que l'on reduit à max-1
    if (chevauchement_arriere_nb > 0) {
        //on merge la balise 1 et la dernière (last)
        for (int i = 2; i < 9; i++) {
            if (tab[i + 9 * (final_nb_bots - 1)] == -1) {
                if (c == 0) c = i;
                if (tab[i - c + 2] != -1) {
                    tab[i + 9 * (final_nb_bots - 1)] = tab[i - c + 2];
                    tab[1 + 9 * (final_nb_bots - 1)]++;
                }
            }
        }
        //on decale
        for (int i = 9; i < 9 * max_nb_bots; i++) {
            tab[i - 9] = tab[i];
        }
        //on efface le last
        for (int i = 9 * (max_nb_bots - 1); i < 9 * max_nb_bots; i++) {
            tab[i] = -1;
        }
        final_nb_bots--;
    }

    //TODO ne garder que les balises detectees au dessus de 150mm

    //sauvegarde les logs des 4 balises
    new_values.z1_p = (int8_t) tab[0];
    new_values.z1_n = (int8_t) tab[1];
    new_values.z1_1 = tab[2];
    new_values.z1_2 = tab[3];
    new_values.z1_3 = tab[4];
    new_values.z1_4 = tab[5];
    new_values.z1_5 = tab[6];
    new_values.z1_6 = tab[7];
    new_values.z1_7 = tab[8];

    new_values.z2_p = (int8_t) tab[9];
    new_values.z2_n = (int8_t) tab[10];
    new_values.z2_1 = tab[11];
    new_values.z2_2 = tab[12];
    new_values.z2_3 = tab[13];
    new_values.z2_4 = tab[14];
    new_values.z2_5 = tab[15];
    new_values.z2_6 = tab[16];
    new_values.z2_7 = tab[17];

    new_values.z3_p = (int8_t) tab[18];
    new_values.z3_n = (int8_t) tab[19];
    new_values.z3_1 = tab[20];
    new_values.z3_2 = tab[21];
    new_values.z3_3 = tab[22];
    new_values.z3_4 = tab[23];
    new_values.z3_5 = tab[24];
    new_values.z3_6 = tab[25];
    new_values.z3_7 = tab[26];

    new_values.z4_p = (int8_t) tab[27];
    new_values.z4_n = (int8_t) tab[28];
    new_values.z4_1 = tab[29];
    new_values.z4_2 = tab[30];
    new_values.z4_3 = tab[31];
    new_values.z4_4 = tab[32];
    new_values.z4_5 = tab[33];
    new_values.z4_6 = tab[34];
    new_values.z4_7 = tab[35];

//2. pour chaque balise déterminer la zone centre et le capteur concerné
    //360 / 72 = 5 degrés par zone

    //float zone_begin_angle_deg[9*4] = { 0.5 , 0.5+4.8 , 0.5+4.8+4.8 , 0.5+4.8+4.8+4.8}; //+20
    //float zone_end_angle_deg[9*4] = { 0.5+4.8 , 0.5+4.8+4.8 , 0.5+4.8+4.8+4.8, 0.5+4.8+4.8+4.8}; //+20

    //pour chaque balise detectée
    for (int i = 1; i <= final_nb_bots; i++) {

        int num_zone_begin = tab[0 + 9 * (i - 1)];
        int num_zone_end = num_zone_begin + tab[1 + 9 * (i - 1)] - 1;

        float zone_begin_angle_deg = (num_zone_begin / 4) * 20.0 + 0.5 + ((num_zone_begin % 4) * 4.8);
        float zone_end_angle_deg = (num_zone_end / 4) * 20.0 + 0.5 + ((num_zone_end % 4) * 4.8) + 4.8;

        //determiné l'angle milieu
        float milieu_deg = (zone_begin_angle_deg + zone_end_angle_deg) / 2.0;
        //si depassement audessus des 72 zones, on reduit d'un tour
        if (milieu_deg > 360) { //num_zone_end > 71 &&
            milieu_deg -= 360;
        }

        pos[2 + 3 * (i - 1)] = milieu_deg + decalage_deg;

        Serial.print("CALCUL Balise" + String(i));
        Serial.print(" num_zone_begin = " + String(num_zone_begin));
        Serial.print(" num_zone_end = " + String(num_zone_end));
        Serial.print(" zone_begin_angle_deg = " + String(zone_begin_angle_deg));
        Serial.print(" zone_end_angle_deg = " + String(zone_end_angle_deg));
        Serial.print(" milieu_deg = " + String(milieu_deg));
        Serial.println();
    }

//3. déterminer la distance sur le repère de la balise (= identique au robot)

    //TODO filtrage de la distance sur 2 balises cote à cote, si mm dist alors c'est la meme balise
    //calcul des distances

    //pour chaque balise detectée calcul de la distance

    int offset_mm = 5;
    for (int i = 1; i <= final_nb_bots; i++) {
        int m = 0;
        int sum = 0;
        int min = 9999;
        for (int z = 2 + 9 * (i - 1); z <= 7 + 9 * (i - 1); z++) {
            //moy de tab[2] à tab[7]
            if (tab[z] > 60) {
                if (tab[z] < min) min = tab[z];
                sum += tab[z];
                m++;
            }
        }
        if (m > 2) {
            m--;
            sum -= min;
        }
        float moy = (1.0 * sum / m) + offset_mm;
        dist[i - 1] = moy + 100; //centre à centre

        Serial.print(" moy(center)" + String(i));
        Serial.print("=" + String(dist[i - 1]));
        Serial.print("  ");
    }
    Serial.println();

    //pour chaque balise detectée calcul des coordonnées x,y
    //avec simplification et arrondi trigonometrique avec un seul sin/cos
    for (int i = 1; i <= final_nb_bots; i++) {
        float x = (dist[i - 1]) * cos(pos[2 + 3 * (i - 1)] * PI / 180.0);
        float y = (dist[i - 1]) * sin(pos[2 + 3 * (i - 1)] * PI / 180.0);
        pos[0 + 3 * (i - 1)] = x;
        pos[1 + 3 * (i - 1)] = y;

        Serial.print("coord" + String(i));
        Serial.print(" x=" + String(x));
        Serial.print(" y=" + String(y));
        Serial.print("  ");
    }
    Serial.println();

    //enregistrement

    new_values.x1_mm = (int16_t) pos[0];
    new_values.y1_mm = (int16_t) pos[1];
    new_values.a1_deg = pos[2];
    new_values.x2_mm = (int16_t) pos[3];
    new_values.y2_mm = (int16_t) pos[4];
    new_values.a2_deg = pos[5];
    new_values.x3_mm = (int16_t) pos[6];
    new_values.y3_mm = (int16_t) pos[7];
    new_values.a3_deg = pos[8];
    new_values.x4_mm = (int16_t) pos[9];
    new_values.y4_mm = (int16_t) pos[10];
    new_values.a4_deg = pos[11];
    new_values.d1_mm = dist[0];
    new_values.d2_mm = dist[1];
    new_values.d3_mm = dist[2];
    new_values.d4_mm = dist[3];

    return final_nb_bots;
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

// Gather raw data and convert to output values
    Registers new_values;

//Save data into new_values
    new_values.c1_mm = filteredResult_coll[0];
    new_values.c2_mm = filteredResult_coll[1];
    new_values.c3_mm = filteredResult_coll[2];
    new_values.c4_mm = filteredResult_coll[3];
    new_values.c5_mm = filteredResult_coll[4];
    new_values.c6_mm = filteredResult_coll[5];
    new_values.c7_mm = filteredResult_coll[6];
    new_values.c8_mm = filteredResult_coll[7];

    //Calcul des positions
    float decalage_deg = 0.0;
    new_values.nbDetectedBots = calculPosition(decalage_deg, new_values);

    Serial.print("FRONT: ");
    Serial.print(new_values.c1_mm);
    Serial.print(" ");
    Serial.print(new_values.c2_mm);
    Serial.print(" ");
    Serial.print(new_values.c3_mm);
    Serial.print(" ");
    Serial.print(new_values.c4_mm);
    Serial.print(" BACK: ");
    Serial.print(new_values.c5_mm);
    Serial.print(" ");
    Serial.print(new_values.c6_mm);
    Serial.print(" ");
    Serial.print(new_values.c7_mm);
    Serial.print(" ");
    Serial.println(new_values.c8_mm);
    Serial.print("  FLAGS: ");
    Serial.print(new_values.flags);
    Serial.print(" NBBOTS: ");
    Serial.print(new_values.nbDetectedBots);

    Serial.print(" dxya1: ");
    Serial.print(new_values.x1_mm);
    Serial.print(" ");
    Serial.print(new_values.y1_mm);
    Serial.print(" ");
    Serial.print(new_values.a1_deg);
    Serial.print(" ");
    Serial.print(new_values.d1_mm);
    Serial.print(" dxya2: ");
    Serial.print(new_values.x2_mm);
    Serial.print(" ");
    Serial.print(new_values.y2_mm);
    Serial.print(" ");
    Serial.print(new_values.a2_deg);
    Serial.print(" ");
    Serial.print(new_values.d2_mm);
    Serial.print(" dxya3: ");
    Serial.print(new_values.x3_mm);
    Serial.print(" ");
    Serial.print(new_values.y3_mm);
    Serial.print(" ");
    Serial.print(new_values.a3_deg);
    Serial.print(" ");
    Serial.print(new_values.d3_mm);
    Serial.print(" dxya4: ");
    Serial.print(new_values.x4_mm);
    Serial.print(" ");
    Serial.print(new_values.y4_mm);
    Serial.print(" ");
    Serial.print(new_values.a4_deg);
    Serial.print(" ");
    Serial.print(new_values.d4_mm);

    Serial.println();
    Serial.print("  R1:");
    Serial.print(new_values.z1_p);
    Serial.print("(");
    Serial.print(new_values.z1_n);
    Serial.print("): ");
    Serial.print(new_values.z1_1);
    Serial.print(" ");
    Serial.print(new_values.z1_2);
    Serial.print(" ");
    Serial.print(new_values.z1_3);
    Serial.print(" ");
    Serial.print(new_values.z1_4);
    Serial.print(" ");
    Serial.print(new_values.z1_5);
    Serial.print(" ");
    Serial.print(new_values.z1_6);
    Serial.print(" ");
    Serial.print(new_values.z1_7);
    Serial.print("  R2:");
    Serial.print(new_values.z2_p);
    Serial.print("(");
    Serial.print(new_values.z2_n);
    Serial.print("): ");
    Serial.print(new_values.z2_1);
    Serial.print(" ");
    Serial.print(new_values.z2_2);
    Serial.print(" ");
    Serial.print(new_values.z2_3);
    Serial.print(" ");
    Serial.print(new_values.z2_4);
    Serial.print(" ");
    Serial.print(new_values.z2_5);
    Serial.print(" ");
    Serial.print(new_values.z2_6);
    Serial.print(" ");
    Serial.print(new_values.z2_7);
    Serial.print("  R3:");
    Serial.print(new_values.z3_p);
    Serial.print("(");
    Serial.print(new_values.z3_n);
    Serial.print("): ");
    Serial.print(new_values.z3_1);
    Serial.print(" ");
    Serial.print(new_values.z3_2);
    Serial.print(" ");
    Serial.print(new_values.z3_3);
    Serial.print(" ");
    Serial.print(new_values.z3_4);
    Serial.print(" ");
    Serial.print(new_values.z3_5);
    Serial.print(" ");
    Serial.print(new_values.z3_6);
    Serial.print(" ");
    Serial.print(new_values.z3_7);
    Serial.print("  R4:");
    Serial.print(new_values.z4_p);
    Serial.print("(");
    Serial.print(new_values.z4_n);
    Serial.print("): ");
    Serial.print(new_values.z4_1);
    Serial.print(" ");
    Serial.print(new_values.z4_2);
    Serial.print(" ");
    Serial.print(new_values.z4_3);
    Serial.print(" ");
    Serial.print(new_values.z4_4);
    Serial.print(" ");
    Serial.print(new_values.z4_5);
    Serial.print(" ");
    Serial.print(new_values.z4_6);
    Serial.print(" ");
    Serial.print(new_values.z4_7);
    Serial.println();

// Block copy new values over the top of the old values
// and then set the "new data" bit.
    memcpy(&registers, &new_values, sizeof(Registers));
    registers.flags = 1;

    tof_loop( false);//registers,

    ledPanels_loop(false);
}
