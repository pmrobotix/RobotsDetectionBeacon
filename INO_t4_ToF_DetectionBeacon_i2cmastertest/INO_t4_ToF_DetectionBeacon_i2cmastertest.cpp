// Do not remove the include below
#include "INO_t4_ToF_DetectionBeacon_i2cmastertest.h"

// Copyright © 2019-2020 Richard Gemmell
// Released under the MIT License. See license.txt. (https://opensource.org/licenses/MIT)

// This example WILL NOT work unless you have an INA260
// current sensor connected to pins 18 and 19.
//
// Demonstrates use of the I2C Device class to represent a slave device.
// Creates an I2C master, configures a device and reads registers.
//
// I recommend this approach for using the Teensy as an I2C slave.

#include <Arduino.h>
#include <i2c_device.h>

// The I2C device. Determines which pins we use on the Teensy.
I2CMaster& master = Master;

// Blink the LED to make sure the Teensy hasn't hung
IntervalTimer blink_timer;
volatile bool led_high = false;

// The slave is an INA 260 current sensor
const uint8_t slave_address = 0x2D;
//const uint8_t manufacturer_id_register = 0xFE;
//const uint16_t expected_manufacturer_id = 0x5449;
//const uint8_t die_id_register = 0xFF;
//const uint16_t expected_die_id = 0x2270;
bool configured = true;
I2CDevice sensor = I2CDevice(master, slave_address, _LITTLE_ENDIAN);

bool configure_sensor();
void report_error(const char* message);

void setup() {
    delay(3000);
    pinMode(LED_BUILTIN, OUTPUT);

    // Initialise the master
    master.begin(100 * 1000U);

    // Enable the serial port for debugging
    Serial.begin(9600);
    Serial.println("Started");

    // Check that we can see the sensor and configure it.
    //configured = configure_sensor();
}

void loop() {
    if (configured) {

        /*
         int16_t d1_mm = 0;
         if (!sensor.read(0x06, &d1_mm, false)) {
         report_error("ERROR: Failed to read d1_mm");
         }
         Serial.printf(" d1_mm: %d ", d1_mm);


         int16_t d2_mm = 0;
         if (!sensor.read(0x08, &d2_mm, true)) {
         report_error("ERROR: Failed to read d2_mm");
         }
         Serial.printf(" d2_mm: %d \n", d2_mm);

         Serial.println();
         */
        /*
         * uint8_t buf[18] = {0};
         if (!sensor.read(0x04, buf, 18, false)) {
         report_error("ERROR: Failed to read collision");
         }

         //        for (int i = 0; i < 18; i++) {
         //            Serial.printf("%02X ", (unsigned char) buf[i]);
         //        }
         //        Serial.println();

         int8_t flag = buf[0];
         int8_t nb = buf[1];
         int16_t c1 = buf[2] | (buf[3] << 8);
         int16_t c2 = buf[4] | (buf[5] << 8);
         int16_t c3 = buf[6] | (buf[7] << 8);
         int16_t c4 = buf[8] | (buf[9] << 8);
         int16_t c5 = buf[10] | (buf[11] << 8);
         int16_t c6 = buf[12] | (buf[13] << 8);
         int16_t c7 = buf[14] | (buf[15] << 8);
         int16_t c8 = buf[16] | (buf[17] << 8);

         Serial.printf("=> %d _ %d _ %d %d %d %d  %d %d %d %d", flag, nb, c1,c2,c3,c4,c5,c6,c7,c8);
         Serial.println();
         */

        Registers regs;

        {
            unsigned char buf[18] = { 0 };
            //TOTAL 12ms à 100kHz
            if (!sensor.read(04, buf, 18, false)) {
                report_error("ERROR: Failed to read collision");
            }

            regs.flags = buf[0];
            regs.nbDetectedBots = buf[1];
            regs.c1_mm = buf[2] | (buf[3] << 8);
            regs.c2_mm = buf[4] | (buf[5] << 8);
            regs.c3_mm = buf[6] | (buf[7] << 8);
            regs.c4_mm = buf[8] | (buf[9] << 8);
            regs.c5_mm = buf[10] | (buf[11] << 8);
            regs.c6_mm = buf[12] | (buf[13] << 8);
            regs.c7_mm = buf[14] | (buf[15] << 8);
            regs.c8_mm = buf[16] | (buf[17] << 8);

//         regs.x1_mm = buf[20] | (buf[21] << 8);
//         regs.y1_mm = buf[22] | (buf[23] << 8);

//         float2bytes_t a1;
//         a1.bytes[0] = buf[24];
//         a1.bytes[1] = buf[25];
//         a1.bytes[2] = buf[26];
//         a1.bytes[3] = buf[27];
//         regs.a1_deg = a1.f;

            Serial.printf("=> %d _ %d _ %d %d %d %d  %d %d %d %d", regs.flags, regs.nbDetectedBots, regs.c1_mm, regs.c2_mm, regs.c3_mm, regs.c4_mm,
                    regs.c5_mm, regs.c6_mm, regs.c7_mm, regs.c8_mm);

            Serial.println();

//         Serial.printf("x1=%d, y1=%d ,a1=%f ", regs.x1_mm, regs.y1_mm, a1.f);
//         Serial.println();
        }

        unsigned char buf[8] = { 0 };
        if (!sensor.read(24, buf, 8, false)) {
            report_error("ERROR: Failed to read testf_buf");
        }

        regs.x1_mm = buf[0] | (buf[1] << 8);
        regs.y1_mm = buf[2] | (buf[3] << 8);
        float2bytes_t a1;
        a1.bytes[0] = buf[4];
        a1.bytes[1] = buf[5];
        a1.bytes[2] = buf[6];
        a1.bytes[3] = buf[7];
        regs.a1_deg = a1.f;

        Serial.printf("x1=%d, y1=%d, a1=%f ", regs.x1_mm, regs.y1_mm, a1.f);
        Serial.println();
        if (!sensor.read(32, buf, 8, false)) {
            report_error("ERROR: Failed to read testf_buf");
        }
        regs.x2_mm = buf[0] | (buf[1] << 8);
        regs.y2_mm = buf[2] | (buf[3] << 8);
        float2bytes_t a2;
        a2.bytes[0] = buf[4];
        a2.bytes[1] = buf[5];
        a2.bytes[2] = buf[6];
        a2.bytes[3] = buf[7];
        regs.a2_deg = a2.f;

        Serial.printf("x2=%d, y2=%d, a2=%f ", regs.x2_mm, regs.y2_mm, a2.f);
        Serial.println();
        if (!sensor.read(40, buf, 8, false)) {
            report_error("ERROR: Failed to read testf_buf");
        }
        regs.x3_mm = buf[0] | (buf[1] << 8);
        regs.y3_mm = buf[2] | (buf[3] << 8);
        float2bytes_t a3;
        a3.bytes[0] = buf[4];
        a3.bytes[1] = buf[5];
        a3.bytes[2] = buf[6];
        a3.bytes[3] = buf[7];
        regs.a3_deg = a3.f;

        Serial.printf("x3=%d, y3=%d, a3=%f ", regs.x3_mm, regs.y3_mm, a3.f);
        Serial.println();
        if (!sensor.read(48, buf, 8, false)) {
            report_error("ERROR: Failed to read testf_buf");
        }
        regs.x4_mm = buf[0] | (buf[1] << 8);
        regs.y4_mm = buf[2] | (buf[3] << 8);
        float2bytes_t a4;
        a4.bytes[0] = buf[4];
        a4.bytes[1] = buf[5];
        a4.bytes[2] = buf[6];
        a4.bytes[3] = buf[7];
        regs.a3_deg = a4.f;

        Serial.printf("x4=%d, y4=%d, a4=%f ", regs.x4_mm, regs.y4_mm, a4.f);
        Serial.println();
        if (!sensor.read(56, buf, 8, true)) {
                    report_error("ERROR: Failed to read testf_buf");
                }
        regs.d1_mm = buf[0] | (buf[1] << 8);
        regs.d2_mm = buf[2] | (buf[3] << 8);
        regs.d3_mm = buf[4] | (buf[5] << 8);
        regs.d4_mm = buf[6] | (buf[7] << 8);

        Serial.printf("d1_mm=%d, d2_mm=%d, d3_mm=%d , d4_mm=%d ", regs.d1_mm, regs.d2_mm, regs.d3_mm, regs.d4_mm);

        Serial.println();
        Serial.println();

//               testf.bytes[0]= testf_buf[0];
//               testf.bytes[1]= testf_buf[1];
//               testf.bytes[2]= testf_buf[2];
//               testf.bytes[3]= testf_buf[3];
//               Serial.printf("=> %f", testf.f);
//               Serial.println();

        /*
         float2bytes_t f;
         //                f.bytes[0] = buf[22];
         //                f.bytes[1] = buf[23];
         //                f.bytes[2] = buf[24];
         //                f.bytes[3] = buf[25];
         f.f = 78.9;
         regs.a1_deg = f.f;

         regs.x2_mm = buf[28] | (buf[29] << 8);
         regs.y2_mm = buf[30] | (buf[31] << 8);

         float2bytes_t f2;
         f2.bytes[0] = buf[30];
         f2.bytes[1] = buf[31];
         f2.bytes[2] = buf[32];
         f2.bytes[3] = buf[33];
         regs.a2_deg = f2.f;


         Serial.printf(" / [%d %d %f %f] [%d %d %f %f]", regs.x1_mm, regs.y1_mm, regs.a1_deg, f.f, regs.x2_mm, regs.y2_mm);//, f2.f, regs.a2_deg);

         Serial.println();
         */
        /*ok
         int16_t current = 0;
         if (!sensor.read(32, &current, false)) {
         report_error("ERROR: Failed to read current");
         }
         Serial.printf("Current: %d \n", current);
         Serial.println();

         unsigned char buf[4] = { 0 };
         if (!sensor.read(36, buf, 4, true)) {
         report_error("ERROR: Failed to read buf");
         }
         Serial.printf("temp : 0x %02x %02x %02x %02x", (unsigned char)buf[0], (unsigned char)buf[1], (unsigned char)buf[2], (unsigned char)buf[3]);
         Serial.println();
         float2bytes_t f;
         f.bytes[0]= buf[0];
         f.bytes[1]= buf[1];
         f.bytes[2]= buf[2];
         f.bytes[3]= buf[3];

         //       f.bytes[0]= 0x36;
         //      f.bytes[1]= 0x42;
         //      f.bytes[2]= 0x6e;
         //      f.bytes[3]= 0x00;
         // f.f = 45.5f;
         Serial.printf("temp : 0x %02x %02x %02x %02x", (unsigned char)f.bytes[0], (unsigned char)f.bytes[1], (unsigned char)f.bytes[2], (unsigned char)f.bytes[3]);
         Serial.println();

         Serial.printf("=> %f", f.f);
         Serial.println();
         */

        /*
         int32_t voltage = buf[4] | (buf[5] << 8) | (buf[6] << 16) << (buf[7] << 24);

         Serial.printf("Voltage: %d mV  ", (voltage));

         float2bytes_t f;
         f.bytes[0] = buf[12];
         f.bytes[1] = buf[13];
         f.bytes[2] = buf[14];
         f.bytes[3] = buf[15];

         Serial.printf("float: %f ", f.f);
         Serial.println();

         int8_t temp_offset = 0;
         if (!sensor.read(0x00, &temp_offset, true)) {
         report_error("ERROR: Failed to read settings");
         }
         Serial.printf("temp_offset: %d ", temp_offset);
         //Serial.println();
         int8_t scaling = 0;
         if (!sensor.read(0x01, &scaling, true)) {
         report_error("ERROR: Failed to read scaling");
         }
         Serial.printf(" scaling: %d \n", scaling);
         Serial.println();


         int8_t tempe = 0;
         if (!sensor.read(0x01, &tempe, true)) {
         report_error("ERROR: Failed to read tempe");
         }
         Serial.printf("tempe : %d deg\n", tempe);
         Serial.println();

         int8_t temp6 = 0;
         int8_t temp7 = 0;
         int8_t temp8 = 0;
         int8_t temp9 = 0;
         if (!sensor.read(0x06, &temp6, true)) {
         report_error("ERROR: Failed to read temp");
         }
         if (!sensor.read(0x07, &temp7, true)) {
         report_error("ERROR: Failed to read temp");
         }
         if (!sensor.read(0x08, &temp8, true)) {
         report_error("ERROR: Failed to read temp");
         }
         if (!sensor.read(0x09, &temp9, true)) {
         report_error("ERROR: Failed to read temp");
         }
         Serial.printf("temp : 0x %02x %02x %02x %02x", (unsigned char)temp6, (unsigned char)temp7, (unsigned char)temp8, (unsigned char)temp9);
         Serial.println();

         int32_t current = 0;
         if (!sensor.read(0x0A, &current, true)) {
         report_error("ERROR: Failed to read current");
         }
         Serial.printf("Current: %d mA\n", current );
         Serial.println();

         int32_t voltage = 0;
         if (!sensor.read(0x06, &voltage, true)) {
         report_error("ERROR: Failed to read voltage");
         }
         Serial.printf("Voltage: %d mV\n", (voltage) );
         Serial.println();
         */
    }
    else {
        Serial.println("Not configured");
    }

    // Blink the LED
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
}

bool configure_sensor() {
    /*
     // Check the manufacturer ID and check for I2C errors
     uint16_t manufacturerId;
     if (sensor.read(manufacturer_id_register, &manufacturerId, false)) {
     if (manufacturerId != expected_manufacturer_id) {
     Serial.printf("ERROR: Manufacturer ID is 0x%X. Expected 0x%X.\n", manufacturerId, expected_manufacturer_id);
     return false;
     }
     } else {
     report_error("ERROR: Failed to send manufacturer id register value");
     return false;
     }

     // Check the Die ID without checking for I2C errors
     uint16_t dieId = 0;
     sensor.read(die_id_register, &dieId, false);
     if (dieId != expected_die_id) {
     Serial.printf("ERROR: Die ID is 0x%X. Expected 0x%X.\n", dieId, expected_die_id);
     return false;
     }

     // Configure it
     uint16_t new_config = 0x6127U | 0x100U | 0x80U | 0x18U;
     if (!sensor.write(0x00, new_config, false)) {
     report_error("ERROR: Failed to set configuration register");
     return false;
     }
     if (!sensor.write(0x06, (uint16_t)0x0403, true)) {
     report_error("ERROR: Failed to set Mask/Enable Register");
     return false;
     }*/

    int8_t numOfBots = 3;
    if (!sensor.write(0x00, numOfBots, false)) {
        report_error("ERROR: Failed to set configuration numOfBots");
        return false;
    }

    int8_t ledDisplay = 100;
    if (!sensor.write(0x01, ledDisplay, true)) {
        report_error("ERROR: Failed to set configuration ledDisplay");
        return false;
    }

    Serial.println("Configured sensor successfully.");
    return true;
}

void report_error(const char* message) {
    Serial.print(message);
    Serial.print(" Error: ");
    Serial.println((int) master.error());
}
