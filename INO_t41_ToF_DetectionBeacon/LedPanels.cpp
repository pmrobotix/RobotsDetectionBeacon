/*
 * LedPanels.cpp
 *
 *  Created on: Jan 25, 2021
 *      Author: cho (PM-ROBOTIX)
 */

#include "INO_ToF_DetectionBeacon.h"
#include "LedPanels.h"
#include <i2c_driver_wire.h>//
//#include <Wire.h>

extern int16_t filteredResult[NumOfZonesPerSensor * NumOfSensors];
extern int16_t distance_t[NumOfZonesPerSensor * NumOfSensors];
extern int16_t status_t[NumOfZonesPerSensor * NumOfSensors];
extern bool connected_t[NumOfZonesPerSensor * NumOfSensors];
extern int16_t SigPerSPAD_t[NumOfZonesPerSensor * NumOfSensors];

extern bool connected_coll[NumOfCollisionSensors + NumOfCollisionSensors];
extern int16_t distance_coll[NumOfCollisionSensors + NumOfCollisionSensors];
extern int16_t status_coll[NumOfCollisionSensors + NumOfCollisionSensors];
extern int16_t SigPerSPAD_coll[NumOfZonesPerSensor * NumOfSensors];

extern bool wait_TofVLReady;
extern int videoMode;

// cLEDMatrix defines
cLEDMatrix<MATRIX_TILE_WIDTH, MATRIX_TILE_HEIGHT, VERTICAL_ZIGZAG_MATRIX,
MATRIX_TILE_H, MATRIX_TILE_V, VERTICAL_BLOCKS> ledmatrix;

// Normally we would define this:
//CRGB leds[NUMMATRIX];

// cLEDMatrix creates a FastLED array inside its object and we need to retrieve
// a pointer to its first element to act as a regular FastLED array, necessary
// for NeoMatrix and other operations that may work directly on the array like FadeAll.
CRGB *leds = ledmatrix[0];

FastLED_NeoMatrix *matrix = new FastLED_NeoMatrix(leds, MATRIX_TILE_WIDTH, MATRIX_TILE_HEIGHT,
MATRIX_TILE_H, MATRIX_TILE_V,
NEO_MATRIX_BOTTOM + NEO_MATRIX_LEFT +
NEO_MATRIX_COLUMNS + NEO_MATRIX_ZIGZAG +
NEO_TILE_BOTTOM + NEO_TILE_LEFT + NEO_TILE_COLUMNS + NEO_TILE_PROGRESSIVE);

bool matrix_reset_demo = 1;
int8_t matrix_loop = -1;

void matrix_clear() {
    // FastLED.clear does not work properly with multiple matrices connected via parallel inputs
    // on ESP8266 (not sure about other chips).
    memset(leds, 0, NUMMATRIX * 3);
}

//Attention pas de delay dans cette fonction pour ne pas ralentir la main loop
void ledPanels_loop(int debug) {

//    if(debug)
//    {
//        //print
//
//    }
    threads.yield();
}

void showFirstLed() {
    //FastLed ligne
    matrix_clear();
    ledmatrix.DrawLine(0, 0, 3, 7, CRGB(0, 255, 0));
    ledmatrix.DrawLine(0, 0, ledmatrix.Width() - 1, ledmatrix.Height() - 1, CRGB(0, 0, 255));
    ledmatrix.DrawLine(0, 0, 7, 7, CRGB(255, 0, 0));
    FastLED.show();
}

void display_connected_vl() {
    //matrix->clear();
    for (int n = 0; n < NumOfSensors; n++) {
        for (int z = 0; z < NumOfZonesPerSensor; z++) {

            int index_led = (int) (((NumOfZonesPerSensor * n) + z) / 2.0);

            if (connected_t[(NumOfZonesPerSensor * n) + z]) {
                matrix->writePixel(index_led, 0, LED_BLUE_MEDIUM);
            }
            else {
                matrix->writePixel(index_led, 0, LED_RED_MEDIUM);
            }
            //                Serial.print(filteredResult[(NumOfZonesPerSensor * n) + z]);
            //                Serial.print(",");
            //matrix->show();
            //threads.delay(5);
        }
    }

}

void ledPanels_setup() {

    // Normal output, no parallel configuration
    FastLED.addLeds<WS2812SERIAL, PIN, BRG>(leds, NUMMATRIX).setCorrection(TypicalLEDStrip);

    // Time for serial port to work?
    //delay(1000);
    //Serial.begin(115200);
    Serial.print("Matrix Size: ");
    Serial.print(mw);
    Serial.print(" * ");
    Serial.println(mh);
    matrix->begin();
    matrix->setTextWrap(false);
    matrix->setBrightness(BRIGHTNESS);
    // Mix in an init of LEDMatrix
    //sprite_setup();
    //display_rgbBitmap(1);
    //threads.delay(1500);

    ledPanels_setup2();
}

void ledPanels_setup2() {
//delay(2000);
//Serial.println("If the code crashes here, decrease the brightness or turn off the all white display below");
// Test full bright of all LEDs. If brightness is too high
// for your current limit (i.e. USB), decrease it.
#ifndef DISABLE_WHITE
    matrix->fillScreen(LED_WHITE_HIGH);
    matrix->show();
    Serial.println("First matrix->show did not crash/hang, trying clear");
    threads.delay(3000);
    matrix_clear();
    Serial.println("First matrix_clear done");
#endif

    showFirstLed();
    threads.delay(1000);

    threads.addThread(display_leds_thread);
}

void display_leds_thread() {

    Serial.println("display_leds_thread starting...");
    //wait for config VL
    while (!wait_TofVLReady) {

        display_panOrBounceBitmap(8);

        //threads.delay(500);
        threads.yield();
    }
    matrix->clear();

    //affichage de la bonne configuration des VL
    //display_connected_vl();
    //threads.delay(1000);

    //int t = 0;
    while (1) {
        if (videoMode == 0)
        {
            matrix->clear();
#ifdef DEBUG_VL_SUR_LEDMATRIX
            add_display_debug();
#endif
            add_display_dist();
            matrix->show();

            threads.yield();
        }
        if (videoMode == 1)
        {
            matrix->clear();
            display_scrollRgbBitmap();
            //display_INVscrollText("Hello I'm PMX!");
            display_INVscrollTextWithBitmap("Don't touch!", 0 , 30);
            matrix->show();
            //threads.delay(2000);
            videoMode = 0;
        }
        //TEST LEDS
        //t++;
        //Serial.println("Count pixels");
        //count_pixels();
        //Serial.println("Count pixels done");
        //delay(400);
        //threads.yield();
        //count_pixels_on_by_one();
        //flag();
        //display_boxes();
        //display_scrollText();

//        display_panOrBounceBitmap(8);
//        threads.delay(2000);
//        display_resolution();
//        threads.delay(2000);
        //display_text("PMX");
        /*
         display_CountInv(5);
         //threads.delay(2000);
         display_scrollRgbBitmap();
         //display_INVscrollText("Happy Birthday Christophe!");
         display_INVscrollTextWithBitmap("Happy Birthday Christophe!", 0);
         */
        //for (int tt = 0; tt < 10000; tt++) {
//            matrix->clear();
//            //display_rgbBitmap(0);
//
//            add_display_dist();
//            matrix->show();
//            threads.yield();
//            threads.delay(1);
//        }
        //display_scrollText_old();
        //threads.delay(2000);
        //sprite_setup();
        //display_lines();
        //display_resolution();
        //display_four_white();
        //display_circles();
        //display_text("PMX");
        //font_loop();
        //threads.delay(2000);
//        Serial.println(t);
        /*
         Serial.println("Use LEDMatrix to display a flag");
         flag();
         delay(3000);
         matrix_clear();

         Serial.println("Back to NeoMatrix/GFX to Display lines, boxes and circles");
         display_lines();
         delay(3000);

         display_boxes();
         delay(3000);

         display_circles();
         delay(3000);*/
        //threads.yield();
        //threads.delay(10);
    }
}
void add_display_debug() {
/*
    //Debug BLEU/RED sur les detecteurs
    for (int n = 0; n < NumOfSensors; n++) {
        for (int z = 0; z < NumOfZonesPerSensor; z++) {

            int index_led = (int) (((NumOfZonesPerSensor * n) + z) / 2.0);

            if (connected_t[(NumOfZonesPerSensor * n) + z]) {
                matrix->writePixel(index_led, 7, LED_BLUE_MEDIUM);
            }
            else {
                matrix->writePixel(index_led, 7, LED_RED_MEDIUM);
            }

        }
    }*/
    int index_led =0;
    int c = 0;
    //Debug //BLEU/RED sur les detecteurs de collision
    for (int n = 0; n < NumOfCollisionSensors + NumOfCollisionSensors; n++) {
        //disposition en ligne
//        int index_led = 28 + n;
//        if (connected_coll[n]) {
//            matrix->writePixel(index_led, 6, LED_BLUE_MEDIUM);
//        }
//        else {
//            matrix->writePixel(index_led, 6, LED_RED_MEDIUM);
//        }
        //disposition en colonnes
        if (n==0){index_led=28;c=3;}
        if (n==1){index_led=29;c=3;}
        if (n==2){index_led=30;c=3;}
        if (n==3){index_led=31;c=3;}
        if (n==4){index_led=28;c=7;}
        if (n==5){index_led=29;c=7;}
        if (n==6){index_led=30;c=7;}
        if (n==7){index_led=31;c=7;}

        if (connected_coll[n]) {
            int val = -1;
            if((status_coll[n] == 0||status_coll[n] == 2)&& SigPerSPAD_coll[n])
                val = distance_coll[n];

            int l=map(val, 50, 400, 0, 3);
            //matrix->writePixel(index_led, c, LED_BLUE_MEDIUM);
            matrix->writePixel(index_led, c-l, LED_BLUE_MEDIUM);
        }
        else {
            matrix->writePixel(index_led, c, LED_RED_MEDIUM);
        }
    }
}
void add_display_dist() {

    matrix->startWrite();

    //GREEN display
    for (int n = 0; n < NumOfZonesPerSensor * NumOfSensors; n = n + 2) {
        //map
        int val = -1;
        int val2 = -1;
        if ((status_t[n] == 0 || status_t[n] ==2)&& SigPerSPAD_t[n] >1000) val = distance_t[n];
        if ((status_t[n + 1] == 0 || status_t[n+1] ==2)&& SigPerSPAD_t[n + 1] >1000) val2 = distance_t[n + 1];
        int maxval=max(val, val2);

        //int maxval = distance_t[n];
        int led_dist = map(maxval, 100, 900, 0, 7);
        if (led_dist < 0) led_dist = 0;
        if (led_dist > 7) led_dist = 7;
        int x_decal = (n / 2) + 2;
        if (x_decal >= 36) x_decal = x_decal - 36;
        if(maxval > 100 && maxval < 1000)
        {
            matrix->writePixel(x_decal, led_dist, LED_GREEN_MEDIUM);
            //matrix->writeLine(x_decal, 0, x_decal, led_dist, LED_GREEN_MEDIUM);
        }
    }

    //Affichage PMX + Distance
    int moyval = 0;
    int moy_x = 0;
    int nb = 0;
    for (int n = 0; n < NumOfZonesPerSensor * NumOfSensors; n = n + 2) {
        //map

        if (filteredResult[n] != -1) {
            //int val=max(filteredResult[n],filteredResult[n+1]);//max
            nb++;
            moyval += filteredResult[n];
            moy_x += n;

        }
        if(n==(NumOfZonesPerSensor * NumOfSensors-1))
        {
            if (filteredResult[0] != -1) {
                //int val=max(filteredResult[n],filteredResult[n+1]);//max
                nb++;
                moyval += filteredResult[0];
                //moy_x += 0;

            }
        }else{
            if (filteredResult[n + 1] != -1) {
                //int val=max(filteredResult[n],filteredResult[n+1]);//max
                nb++;
                moyval += filteredResult[n + 1];
                moy_x += n + 1;
            }
        }
    }

    if (nb > 0) {
        moyval = 1.0 * moyval / nb;
        moy_x = 1.0 * moy_x / nb;
        int led_dist = map(moyval, 30, 1500, 0, 7);
        if (led_dist < 0) led_dist = 0;
        if (led_dist > 7) led_dist = 7;
        int x_decal = (moy_x / 2) + 2; //+2=Nb de declage de zone entre les VL et les leds sur la balise
        if (x_decal >= 36) x_decal = x_decal - 36;

        if (led_dist > 2) write_PMX(x_decal, 0, LED_BLUE_HIGH);
        else if (led_dist > 1 && led_dist <= 2) write_PMX(x_decal, 0, LED_PURPLE_HIGH);
        else write_PMX(x_decal, 0, LED_RED_HIGH);

        int x_opp = x_decal + 18;
        if (x_opp >= 36) x_opp = x_opp - 36;
        //matrix->writePixel(x_opp, 7-led_dist, LED_RED_HIGH);

        if (led_dist < 1) write_PMX(x_opp, 0, LED_RED_HIGH);
        else {

            //affichage distance
            //seuillage à 1cm pres
            int moyval10cm = moyval / 100.0;

            matrix->setTextSize(1);
            matrix->setFont();
            matrix->setRotation(0);
            matrix->setTextWrap(false);
            matrix->setTextColor(matrix->Color(255, 100, 0));
            matrix->setCursor(x_opp - 2, 0);
            matrix->print(moyval10cm);

            //affichage du pixel
            matrix->writePixel(x_opp, 7 - led_dist, LED_RED_HIGH);
        }

    }
    String tmp = "";
    //test affichage du filtered
    for (int n = 0; n < NumOfZonesPerSensor * NumOfSensors; n = n + 2) {
        //map
        int val = filteredResult[n];
        int val2 = -1;
        if(n<=NumOfZonesPerSensor * NumOfSensors -1)
            val2 = filteredResult[n+1];
        else
            val2 = filteredResult[0];

//        if (status_t[n] == 0 || status_t[n] == 2) val = distance_t[n];
//        if (status_t[n + 1] == 0 || status_t[n + 1] == 2) val2 = distance_t[n + 1];
        int maxval=max(val, val2);
        tmp +=maxval + " ";

        //int maxval = distance_t[n];
//        int led_dist = map(maxval, 20, 700, 0, 7);
//        if (led_dist < 0) led_dist = 0;
//        if (led_dist > 7) led_dist = 7;
        int x_decal = (n / 2) + 2;
        if (x_decal >= 36) x_decal = x_decal - 36;
        int x_opp = x_decal + 18;
        if (x_opp >= 36) x_opp = x_opp - 36;
        //if(maxval > 100 && maxval < 1000)
        if (maxval>0)
            matrix->writePixel(x_opp, 7, LED_WHITE_MEDIUM);
    }
    //Serial.println(tmp);
    matrix->endWrite();

}

void write_PMX(int x, int y, uint32_t color) {
    matrix->writePixel(x, y, color);
    matrix->writePixel(x - 1, y + 1, color);
    matrix->writeLine(x - 2, y + 2, x + 2, y + 2, color);
    matrix->writeLine(x - 2, y + 6, x + 2, y + 6, color);
    matrix->writeLine(x - 3, y + 3, x - 3, y + 5, color);
    matrix->writeLine(x + 3, y + 3, x + 3, y + 5, color);
    matrix->writePixel(x + 1, y + 4, color);
    matrix->writePixel(x - 1, y + 4, color);
}

void display_dist() {
    matrix_clear();
    //ledmatrix.DrawLine(0, 0, ledmatrix.Width() - 1, ledmatrix.Height() - 1, CRGB(0, 255, 255));
    //matrix->drawLine(0, 0, mw - 1, mh - 1, LED_BLUE_HIGH);
    matrix->drawLine(0, mh - 1, mw - 1, 0, CRGB(0, 255, 255));

    //matrix->drawCircle(mw / 2, mh / 2, 2, LED_RED_MEDIUM);
    matrix->show();

}
//uint32_t PMX[8][8] = {
//  {BLK, BLK, BLK, BLK, WHI, BLK, BLK, BLK},
//  {BLK, BLK, BLK, WHI, BLK, BLK, BLK, BLK},
//  {BLK, WHI, WHI, WHI, WHI, WHI, WHI, BLK},
//  {WHI, BLK, BLK, BLK, BLK, BLK, BLK, WHI},
//  {WHI, BLK, WHI, BLK, BLK, WHI, BLK, WHI},
//  {WHI, BLK, BLK, BLK, BLK, BLK, BLK, WHI},
//  {BLK, WHI, WHI, WHI, WHI, WHI, WHI, BLK},
//  {BLK, BLK, BLK, BLK, BLK, BLK, BLK, BLK}
//};
static const uint16_t PROGMEM
// These bitmaps were written for a backend that only supported
// 4 bits per color with Blue/Green/Red ordering while neomatrix
// uses native 565 color mapping as RGB.
// I'm leaving the arrays as is because it's easier to read
// which color is what when separated on a 4bit boundary
// The demo code will modify the arrays at runtime to be compatible
// with the neomatrix color ordering and bit depth.
RGB_bmp[][64] = {
// 00: PM-ROBOTIX 8pixels yeux normaux
        { 0x000, 0x000, 0x000, 0x000, 0xF00, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0xF00, 0x000, 0x000, 0x000, 0x000, 0x000, 0x800, 0x800, 0xF00,
                0x800, 0x800, 0x800, 0x000, 0x800, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x800, 0x800, 0x000, 0xF00, 0x000, 0x000, 0xF00, 0x000,
                0x800, 0x800, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x800, 0x000, 0x800, 0x800, 0x800, 0x800, 0x800, 0x800, 0x000, 0x000, 0x000,
                0x000, 0x000, 0x000, 0x000, 0x000, 0x000, },

        // 01: PM-ROBOTIX 8pixels yeux rouges
        { 0x000, 0x000, 0x000, 0x000, 0xF00, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0xF00, 0x000, 0x000, 0x000, 0x000, 0x000, 0xA00, 0xA00, 0xA00,
                0xA00, 0xA00, 0xA00, 0x000, 0xA00, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0xA00, 0xA00, 0x000, 0x009, 0x000, 0x000, 0x009, 0x000,
                0xA00, 0xA00, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0xA00, 0x000, 0xA00, 0xA00, 0xA00, 0xA00, 0xA00, 0xA00, 0x000, 0x000, 0x000,
                0x000, 0x000, 0x000, 0x000, 0x000, 0x000, },
        // 02: PM-ROBOTIX 7pixels yeux rouges
        { 0x000, 0x000, 0x000, 0x000, 0xF00, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0xF00, 0x000, 0x000, 0x000, 0x000, 0x000, 0xA00, 0xA00, 0xA00,
                0xA00, 0xA00, 0x000, 0x000, 0xA00, 0x000, 0x000, 0x000, 0x000, 0x000, 0xA00, 0x000, 0xA00, 0x000, 0x009, 0x000, 0x009, 0x000, 0xA00,
                0x000, 0xA00, 0x000, 0x000, 0x000, 0x000, 0x000, 0xA00, 0x000, 0x000, 0xA00, 0xA00, 0xA00, 0xA00, 0xA00, 0x000, 0x000, 0x000, 0x000,
                0x000, 0x000, 0x000, 0x000, 0x000, 0x000, },

        // 00: blue, blue/red, red, red/green, green, green/blue, blue, white
        { 0x100, 0x200, 0x300, 0x400, 0x600, 0x800, 0xA00, 0xF00, 0x101, 0x202, 0x303, 0x404, 0x606, 0x808, 0xA0A, 0xF0F, 0x001, 0x002, 0x003, 0x004,
                0x006, 0x008, 0x00A, 0x00F, 0x011, 0x022, 0x033, 0x044, 0x066, 0x088, 0x0AA, 0x0FF, 0x010, 0x020, 0x030, 0x040, 0x060, 0x080, 0x0A0,
                0x0F0, 0x110, 0x220, 0x330, 0x440, 0x660, 0x880, 0xAA0, 0xFF0, 0x100, 0x200, 0x300, 0x400, 0x600, 0x800, 0xA00, 0xF00, 0x111, 0x222,
                0x333, 0x444, 0x666, 0x888, 0xAAA, 0xFFF, },

        // 01: grey to white
        { 0x111, 0x222, 0x333, 0x555, 0x777, 0x999, 0xAAA, 0xFFF, 0x222, 0x222, 0x333, 0x555, 0x777, 0x999, 0xAAA, 0xFFF, 0x333, 0x333, 0x333, 0x555,
                0x777, 0x999, 0xAAA, 0xFFF, 0x555, 0x555, 0x555, 0x555, 0x777, 0x999, 0xAAA, 0xFFF, 0x777, 0x777, 0x777, 0x777, 0x777, 0x999, 0xAAA,
                0xFFF, 0x999, 0x999, 0x999, 0x999, 0x999, 0x999, 0xAAA, 0xFFF, 0xAAA, 0xAAA, 0xAAA, 0xAAA, 0xAAA, 0xAAA, 0xAAA, 0xFFF, 0xFFF, 0xFFF,
                0xFFF, 0xFFF, 0xFFF, 0xFFF, 0xFFF, 0xFFF, },

        // 02: low red to high red
        { 0x001, 0x002, 0x003, 0x005, 0x007, 0x009, 0x00A, 0x00F, 0x002, 0x002, 0x003, 0x005, 0x007, 0x009, 0x00A, 0x00F, 0x003, 0x003, 0x003, 0x005,
                0x007, 0x009, 0x00A, 0x00F, 0x005, 0x005, 0x005, 0x005, 0x007, 0x009, 0x00A, 0x00F, 0x007, 0x007, 0x007, 0x007, 0x007, 0x009, 0x00A,
                0x00F, 0x009, 0x009, 0x009, 0x009, 0x009, 0x009, 0x00A, 0x00F, 0x00A, 0x00A, 0x00A, 0x00A, 0x00A, 0x00A, 0x00A, 0x00F, 0x00F, 0x00F,
                0x00F, 0x00F, 0x00F, 0x00F, 0x00F, 0x00F, },

        // 03: low green to high green
        { 0x010, 0x020, 0x030, 0x050, 0x070, 0x090, 0x0A0, 0x0F0, 0x020, 0x020, 0x030, 0x050, 0x070, 0x090, 0x0A0, 0x0F0, 0x030, 0x030, 0x030, 0x050,
                0x070, 0x090, 0x0A0, 0x0F0, 0x050, 0x050, 0x050, 0x050, 0x070, 0x090, 0x0A0, 0x0F0, 0x070, 0x070, 0x070, 0x070, 0x070, 0x090, 0x0A0,
                0x0F0, 0x090, 0x090, 0x090, 0x090, 0x090, 0x090, 0x0A0, 0x0F0, 0x0A0, 0x0A0, 0x0A0, 0x0A0, 0x0A0, 0x0A0, 0x0A0, 0x0F0, 0x0F0, 0x0F0,
                0x0F0, 0x0F0, 0x0F0, 0x0F0, 0x0F0, 0x0F0, },

        // 04: low blue to high blue
        { 0x100, 0x200, 0x300, 0x500, 0x700, 0x900, 0xA00, 0xF00, 0x200, 0x200, 0x300, 0x500, 0x700, 0x900, 0xA00, 0xF00, 0x300, 0x300, 0x300, 0x500,
                0x700, 0x900, 0xA00, 0xF00, 0x500, 0x500, 0x500, 0x500, 0x700, 0x900, 0xA00, 0xF00, 0x700, 0x700, 0x700, 0x700, 0x700, 0x900, 0xA00,
                0xF00, 0x900, 0x900, 0x900, 0x900, 0x900, 0x900, 0xA00, 0xF00, 0xA00, 0xA00, 0xA00, 0xA00, 0xA00, 0xA00, 0xA00, 0xF00, 0xF00, 0xF00,
                0xF00, 0xF00, 0xF00, 0xF00, 0xF00, 0xF00, },

        // 05: 1 black, 2R, 2O, 2G, 1B with 4 blue lines rising right
        { 0x000, 0x200, 0x000, 0x400, 0x000, 0x800, 0x000, 0xF00, 0x000, 0x201, 0x002, 0x403, 0x004, 0x805, 0x006, 0xF07, 0x008, 0x209, 0x00A, 0x40B,
                0x00C, 0x80D, 0x00E, 0xF0F, 0x000, 0x211, 0x022, 0x433, 0x044, 0x855, 0x066, 0xF77, 0x088, 0x299, 0x0AA, 0x4BB, 0x0CC, 0x8DD, 0x0EE,
                0xFFF, 0x000, 0x210, 0x020, 0x430, 0x040, 0x850, 0x060, 0xF70, 0x080, 0x290, 0x0A0, 0x4B0, 0x0C0, 0x8D0, 0x0E0, 0xFF0, 0x000, 0x200,
                0x000, 0x500, 0x000, 0x800, 0x000, 0xF00, },

        // 06: 4 lines of increasing red and then green
        { 0x000, 0x000, 0x001, 0x001, 0x002, 0x002, 0x003, 0x003, 0x004, 0x004, 0x005, 0x005, 0x006, 0x006, 0x007, 0x007, 0x008, 0x008, 0x009, 0x009,
                0x00A, 0x00A, 0x00B, 0x00B, 0x00C, 0x00C, 0x00D, 0x00D, 0x00E, 0x00E, 0x00F, 0x00F, 0x000, 0x000, 0x010, 0x010, 0x020, 0x020, 0x030,
                0x030, 0x040, 0x040, 0x050, 0x050, 0x060, 0x060, 0x070, 0x070, 0x080, 0x080, 0x090, 0x090, 0x0A0, 0x0A0, 0x0B0, 0x0B0, 0x0C0, 0x0C0,
                0x0D0, 0x0D0, 0x0E0, 0x0E0, 0x0F0, 0x0F0, },

        // 07: 4 lines of increasing red and then blue
        { 0x000, 0x000, 0x001, 0x001, 0x002, 0x002, 0x003, 0x003, 0x004, 0x004, 0x005, 0x005, 0x006, 0x006, 0x007, 0x007, 0x008, 0x008, 0x009, 0x009,
                0x00A, 0x00A, 0x00B, 0x00B, 0x00C, 0x00C, 0x00D, 0x00D, 0x00E, 0x00E, 0x00F, 0x00F, 0x000, 0x000, 0x100, 0x100, 0x200, 0x200, 0x300,
                0x300, 0x400, 0x400, 0x500, 0x500, 0x600, 0x600, 0x700, 0x700, 0x800, 0x800, 0x900, 0x900, 0xA00, 0xA00, 0xB00, 0xB00, 0xC00, 0xC00,
                0xD00, 0xD00, 0xE00, 0xE00, 0xF00, 0xF00, },

        // 08: criss cross of green and red with diagonal blue.
        { 0xF00, 0x001, 0x003, 0x005, 0x007, 0x00A, 0x00F, 0x000, 0x020, 0xF21, 0x023, 0x025, 0x027, 0x02A, 0x02F, 0x020, 0x040, 0x041, 0xF43, 0x045,
                0x047, 0x04A, 0x04F, 0x040, 0x060, 0x061, 0x063, 0xF65, 0x067, 0x06A, 0x06F, 0x060, 0x080, 0x081, 0x083, 0x085, 0xF87, 0x08A, 0x08F,
                0x080, 0x0A0, 0x0A1, 0x0A3, 0x0A5, 0x0A7, 0xFAA, 0x0AF, 0x0A0, 0x0F0, 0x0F1, 0x0F3, 0x0F5, 0x0F7, 0x0FA, 0xFFF, 0x0F0, 0x000, 0x001,
                0x003, 0x005, 0x007, 0x00A, 0x00F, 0xF00, },

        // 09: 2 lines of green, 2 red, 2 orange, 2 green
        { 0x0F0, 0x0F0, 0x0FF, 0x0FF, 0x00F, 0x00F, 0x0F0, 0x0F0, 0x0F0, 0x0F0, 0x0FF, 0x0FF, 0x00F, 0x00F, 0x0F0, 0x0F0, 0x0F0, 0x0F0, 0x0FF, 0x0FF,
                0x00F, 0x00F, 0x0F0, 0x0F0, 0x0F0, 0x0F0, 0x0FF, 0x0FF, 0x00F, 0x00F, 0x0F0, 0x0F0, 0x0F0, 0x0F0, 0x0FF, 0x0FF, 0x00F, 0x00F, 0x0F0,
                0x0F0, 0x0F0, 0x0F0, 0x0FF, 0x0FF, 0x00F, 0x00F, 0x0F0, 0x0F0, 0x0F0, 0x0F0, 0x0FF, 0x0FF, 0x00F, 0x00F, 0x0F0, 0x0F0, 0x0F0, 0x0F0,
                0x0FF, 0x0FF, 0x00F, 0x00F, 0x0F0, 0x0F0, },

        // 10: multicolor smiley face
        { 0x000, 0x000, 0x00F, 0x00F, 0x00F, 0x00F, 0x000, 0x000, 0x000, 0x00F, 0x000, 0x000, 0x000, 0x000, 0x00F, 0x000, 0x00F, 0x000, 0xF00, 0x000,
                0x000, 0xF00, 0x000, 0x00F, 0x00F, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x00F, 0x00F, 0x000, 0x0F0, 0x000, 0x000, 0x0F0, 0x000,
                0x00F, 0x00F, 0x000, 0x000, 0x0F4, 0x0F3, 0x000, 0x000, 0x00F, 0x000, 0x00F, 0x000, 0x000, 0x000, 0x000, 0x00F, 0x000, 0x000, 0x000,
                0x00F, 0x00F, 0x00F, 0x00F, 0x000, 0x000, }, };

// Scroll within big bitmap so that all if it becomes visible or bounce a small one.
// If the bitmap is bigger in one dimension and smaller in the other one, it will
// be both panned and bounced in the appropriate dimensions.
void display_panOrBounceBitmap(uint8_t bitmapSize) {
    // keep integer math, deal with values 16 times too big
    // start by showing upper left of big bitmap or centering if the display is big
    int16_t xf = max(0, (mw - bitmapSize) / 2) << 4;
    int16_t yf = max(0, (mh - bitmapSize) / 2) << 4;
    // scroll speed in 1/16th
    int16_t xfc = 6;
    int16_t yfc = 3;
    // scroll down and right by moving upper left corner off screen
    // more up and left (which means negative numbers)
    int16_t xfdir = -1;
    int16_t yfdir = -1;

    for (uint16_t i = 1; i < 200; i++) {
        bool updDir = false;

        // Get actual x/y by dividing by 16.
        int16_t x = xf >> 4;
        int16_t y = yf >> 4;

        matrix->clear();
        // bounce 8x8 tri color smiley face around the screen
        if (bitmapSize == 8) fixdrawRGBBitmap(x, y, RGB_bmp[0], 8, 8);
        // pan 24x24 pixmap
//          if (bitmapSize == 24) matrix->drawRGBBitmap(x, y, (const uint16_t *) bitmap24, bitmapSize, bitmapSize);
//#ifdef BM32
//          if (bitmapSize == 32) matrix->drawRGBBitmap(x, y, (const uint16_t *) bitmap32, bitmapSize, bitmapSize);
//#endif
        matrix->show();

        // Only pan if the display size is smaller than the pixmap
        // but not if the difference is too small or it'll look bad.
        if (bitmapSize - mw > 2) {
            xf += xfc * xfdir;
            if (xf >= 0) {
                xfdir = -1;
                updDir = true;
            };
            // we don't go negative past right corner, go back positive
            if (xf <= ((mw - bitmapSize) << 4)) {
                xfdir = 1;
                updDir = true;
            };
        }
        if (bitmapSize - mh > 2) {
            yf += yfc * yfdir;
            // we shouldn't display past left corner, reverse direction.
            if (yf >= 0) {
                yfdir = -1;
                updDir = true;
            };
            if (yf <= ((mh - bitmapSize) << 4)) {
                yfdir = 1;
                updDir = true;
            };
        }
        // only bounce a pixmap if it's smaller than the display size
        if (mw > bitmapSize) {
            xf += xfc * xfdir;
            // Deal with bouncing off the 'walls'
            if (xf >= (mw - bitmapSize) << 4) {
                xfdir = -1;
                updDir = true;
            };
            if (xf <= 0) {
                xfdir = 1;
                updDir = true;
            };
        }
        if (mh > bitmapSize) {
            yf += yfc * yfdir;
            if (yf >= (mh - bitmapSize) << 4) {
                yfdir = -1;
                updDir = true;
            };
            if (yf <= 0) {
                yfdir = 1;
                updDir = true;
            };
        }

        if (updDir) {
            // Add -1, 0 or 1 but bind result to 1 to 1.
            // Let's take 3 is a minimum speed, otherwise it's too slow.
            xfc = constrain(xfc + random(-1, 2), 3, 16);
            yfc = constrain(xfc + random(-1, 2), 3, 16);
        }
        threads.delay(10);
    }
}

// Convert a BGR 4/4/4 bitmap to RGB 5/6/5 used by Adafruit_GFX
void fixdrawRGBBitmap(int16_t x, int16_t y, const uint16_t *bitmap, int16_t w, int16_t h) {
    uint16_t RGB_bmp_fixed[w * h];
    for (uint16_t pixel = 0; pixel < w * h; pixel++) {
        uint8_t r, g, b;
        uint16_t color = pgm_read_word(bitmap + pixel);

        //Serial.print(color, HEX);
        b = (color & 0xF00) >> 8;
        g = (color & 0x0F0) >> 4;
        r = color & 0x00F;
        //Serial.print(" ");
        //Serial.print(b);
        //Serial.print("/");
        //Serial.print(g);
        //Serial.print("/");
        //Serial.print(r);
        //Serial.print(" -> ");
        // expand from 4/4/4 bits per color to 5/6/5
        b = map(b, 0, 15, 0, 31);
        g = map(g, 0, 15, 0, 63);
        r = map(r, 0, 15, 0, 31);
        //Serial.print(r);
        //Serial.print("/");
        //Serial.print(g);
        //Serial.print("/");
        //Serial.print(b);
        RGB_bmp_fixed[pixel] = (r << 11) + (g << 5) + b;
        //Serial.print(" -> ");
        //Serial.println(RGB_bmp_fixed[pixel], HEX);
    }
    matrix->drawRGBBitmap(x, y, RGB_bmp_fixed, w, h);
}

void display_rgbBitmap(uint8_t bmp_num, uint16_t bmx, uint16_t bmy, bool clearAndShow) {
    //static uint16_t bmx, bmy;
    if (clearAndShow) matrix->clear();

    fixdrawRGBBitmap(bmx, bmy, RGB_bmp[bmp_num], 8, 8);
    bmx += 8;
    if (bmx >= mw) bmx = 0;
    if (!bmx) bmy += 8;
    if (bmy >= mh) bmy = 0;

    if (clearAndShow) matrix->show();
}

// Fill the screen with multiple levels of white to gauge the quality
void display_four_white() {
    matrix->clear();
    matrix->fillRect(0, 0, mw, mh, LED_WHITE_HIGH);
    matrix->drawRect(1, 1, mw - 2, mh - 2, LED_WHITE_MEDIUM);
    matrix->drawRect(2, 2, mw - 4, mh - 4, LED_WHITE_LOW);
    matrix->drawRect(3, 3, mw - 6, mh - 6, LED_WHITE_VERYLOW);
    matrix->show();
}
void display_text(String txt) {
    matrix->clear();

    matrix->setTextSize(1);
    matrix->setFont();
    matrix->setRotation(0);
    matrix->setTextWrap(false);
    matrix->setTextColor(matrix->Color(255, 128, 0));

    matrix->setCursor(0, 0);
    matrix->print(txt);

    add_display_dist();
    matrix->show();
}

void display_resolution() {
    matrix->setTextSize(1);
    // not wide enough;
    if (mw < 16) return;
    matrix->clear();
    // Font is 5x7, if display is too small
    // 8 can only display 1 char
    // 16 can almost display 3 chars
    // 24 can display 4 chars
    // 32 can display 5 chars
    matrix->setCursor(0, 0);
    matrix->setTextColor(matrix->Color(255, 0, 0));
    if (mw > 10) matrix->print(mw / 10);
    matrix->setTextColor(matrix->Color(255, 128, 0));
    matrix->print(mw % 10);
    matrix->setTextColor(matrix->Color(0, 255, 0));
    matrix->print('x');
    // not wide enough to print 5 chars, go to next line
    if (mw < 25) {
        if (mh == 13) matrix->setCursor(6, 7);
        else if (mh >= 13) {
            matrix->setCursor(mw - 11, 8);
        }
        else {
            // we're not tall enough either, so we wait and display
            // the 2nd value on top.
            matrix->show();
            delay(2000);
            matrix->clear();
            matrix->setCursor(mw - 11, 0);
        }
    }
    matrix->setTextColor(matrix->Color(0, 255, 128));
    matrix->print(mh / 10);
    matrix->setTextColor(matrix->Color(0, 128, 255));
    matrix->print(mh % 10);
    // enough room for a 2nd line
    if ((mw > 25 && mh > 14) || mh > 16) {
        matrix->setCursor(0, mh - 7);
        matrix->setTextColor(matrix->Color(0, 255, 255));
        if (mw > 16) matrix->print('*');
        matrix->setTextColor(matrix->Color(255, 0, 0));
        matrix->print('R');
        matrix->setTextColor(matrix->Color(0, 255, 0));
        matrix->print('G');
        matrix->setTextColor(matrix->Color(0, 0, 255));
        matrix->print("B");
        matrix->setTextColor(matrix->Color(255, 255, 0));
        // this one could be displayed off screen, but we don't care :)
        matrix->print("*");

        // We have a big array, great, let's assume 32x32 and add something in the middle
        if (mh > 24 && mw > 25) {
            for (uint16_t i = 0; i < mw; i += 8)
                fixdrawRGBBitmap(i, mh / 2 - 7 + (i % 16) / 8 * 6, RGB_bmp[10], 8, 8);
        }
    }

    matrix->show();
}
void display_CountInv(int fromNumber) {

    matrix->clear();
    matrix->setTextSize(1);
    matrix->setFont();
    matrix->setRotation(0);
    matrix->setTextWrap(false);
    matrix->setFont();

    for (int f = fromNumber; f >= 1; f--) {
        yield();
        matrix->clear();
        matrix->setTextColor(LED_RED_HIGH);
        matrix->setCursor(0, 0);
        matrix->print(f);
        matrix->show();
        threads.delay(500);
        matrix->setTextColor(LED_PURPLE_HIGH);
        for (int x = 1; x <= mw; x++) {
            yield();
            matrix->clear();

            matrix->setCursor(x, 0);
            matrix->print(f);
            matrix->show();
            threads.delay(20);
        }
    }
}

void display_scrollRgbBitmap() {
    display_rgbBitmap(0, 0, 0);
    threads.delay(800);
    for (int x = 0; x <= mw; x++) {
        display_rgbBitmap(0, x, 0);
        threads.delay(20);
    }
}
void display_INVscrollTextWithBitmap(String txt, int bitmap, int delay_ms) {

    matrix->setTextSize(1);
    matrix->setFont();
    matrix->setRotation(0);
    matrix->setTextWrap(false);

    for (int x = mw + 10; x > 0 - (int) (txt.length() * 6); x--) {
        yield();
        matrix->clear();

        display_rgbBitmap(bitmap, x - 9, 0);

        //matrix->setTextColor(matrix->Color(255, 128, 0));
        matrix->setTextColor(x * 10);

        matrix->setCursor(x, 0);
        matrix->print(txt);

        //add_display_dist();

        matrix->show();
        threads.delay(delay_ms);
    }

//matrix->setRotation(0);
//matrix->setCursor(0, 0);
//matrix->show();

}

void display_INVscrollText(String txt, int delay_ms) {
    matrix->setTextSize(1);
    matrix->setFont();
    matrix->setRotation(0);
    matrix->setTextWrap(false);

    for (int x = mw; x > 0 - (int) (txt.length() * 6); x--) {
        yield();
        matrix->clear();

        //matrix->setTextColor(matrix->Color(255, 128, 0));
        matrix->setTextColor(LED_GREEN_HIGH);

        matrix->setCursor(x, 0);
        matrix->print(txt);

        matrix->show();
        threads.delay(75);
    }

//    matrix->setRotation(0);
//    matrix->setCursor(0, 0);
//    matrix->show();
}

void display_scrollText(String txt) {

    matrix->setTextSize(1);
    matrix->setFont();
    matrix->setRotation(0);
    matrix->setTextWrap(false);

    for (int x = 0 - (txt.length() * 6); x <= (mw); x++) {
        yield();
        matrix->clear();

        //matrix->setTextColor(matrix->Color(255, 128, 0));
        matrix->setTextColor(LED_GREEN_HIGH);

        matrix->setCursor(x, 0);
        matrix->print(txt);

        matrix->show();
        threads.delay(75);
    }

//    matrix->setRotation(0);
//    matrix->setCursor(0, 0);
//    matrix->show();

}

void display_scrollText_old() {
    matrix->setFont();
    uint8_t size = max(int(mw / 8), 1);
    matrix->clear();
    matrix->setTextWrap(false);  // we don't wrap text so it scrolls nicely
    matrix->setTextSize(1);
    matrix->setRotation(0);
    for (int8_t x = 1; x >= 42; x++) {
        yield();
        matrix->clear();
        matrix->setCursor(x, 0);
        matrix->setTextColor(LED_GREEN_HIGH);
        matrix->print("Hello");
        if (mh > 11) {
            matrix->setCursor(-20 - x, mh - 7);
            matrix->setTextColor(LED_ORANGE_HIGH);
            matrix->print("World");
        }
        matrix->show();
        threads.delay(50);
    }

    matrix->setRotation(0);
    matrix->setTextSize(size);
    matrix->setTextColor(LED_BLUE_HIGH);
    for (int16_t x = 8 * size; x >= -6 * 8 * size; x--) {
        yield();
        matrix->clear();
        matrix->setCursor(x, mw / 2 - size * 4);
        matrix->print("Rotate");
        matrix->show();
        // note that on a big array the refresh rate from show() will be slow enough that
        // the delay become irrelevant. This is already true on a 32x32 array.
        threads.delay(50 / size);
    }
    matrix->setRotation(0);
    matrix->setCursor(0, 0);
    matrix->show();
}

void sprite_setup() {
    matrix_clear();
    //Serial.println("ledmatrix setup");
    ledmatrix.DrawLine(0, 0, ledmatrix.Width() - 1, ledmatrix.Height() - 1, CRGB(0, 255, 0));
    ledmatrix.DrawPixel(0, 0, CRGB(255, 0, 0));
    ledmatrix.DrawPixel(ledmatrix.Width() - 1, ledmatrix.Height() - 1, CRGB(0, 0, 255));
    FastLED.show();
    //Serial.println("ledmatrix setup done");
}

void flag() {
    matrix_clear();
    ledmatrix.DrawFilledRectangle(0, 0, ledmatrix.Width() - 1, ledmatrix.Height() - 1, CRGB(0, 0, 255));
    ledmatrix.DrawRectangle(0, 0, ledmatrix.Width() - 1, ledmatrix.Height() - 1, CRGB(255, 255, 255));
    ledmatrix.DrawLine(0, 0, ledmatrix.Width() - 1, ledmatrix.Height() - 1, CRGB(255, 255, 255));
    ledmatrix.DrawLine(0, 1, ledmatrix.Width() - 1, ledmatrix.Height() - 2, CRGB(255, 255, 255));
    ledmatrix.DrawLine(0, ledmatrix.Height() - 1, ledmatrix.Width() - 1, 0, CRGB(255, 255, 255));
    ledmatrix.DrawLine(0, ledmatrix.Height() - 2, ledmatrix.Width() - 1, 1, CRGB(255, 255, 255));
    FastLED.show();
}

void count_pixels_on_by_one() {
    matrix_clear();
    for (int i = 0; i < mh; i++) {
        for (int j = 0; j < mw; j++) {
            matrix->drawPixel(j, i, CRGB(255, 0, 0));
            matrix->show();
            threads.yield();
        }
    }

}

// In a case of a tile of neomatrices, this test is helpful to make sure that the
// pixels are all in sequence (to check your wiring order and the tile options you
// gave to the constructor).
void count_pixels() {
    matrix_clear();
    for (uint16_t i = 0; i < mh; i++) {
        for (uint16_t j = 0; j < mw; j++) {
            matrix->drawPixel(j, i, i % 3 == 0 ? (uint16_t) LED_BLUE_HIGH : i % 3 == 1 ? (uint16_t) LED_RED_HIGH : (uint16_t) LED_GREEN_HIGH);
            // depending on the matrix size, it's too slow to display each pixel, so
            // make the scan init faster. This will however be too fast on a small matrix.

            matrix->show();
            threads.yield();
        }
    }
}

void display_lines() {
    matrix_clear();

    // 4 levels of crossing red lines.
    matrix->drawLine(0, mh / 2 - 2, mw - 1, 2, LED_RED_VERYLOW);
    matrix->drawLine(0, mh / 2 - 1, mw - 1, 3, LED_RED_LOW);
    matrix->drawLine(0, mh / 2, mw - 1, mh / 2, LED_RED_MEDIUM);
    matrix->drawLine(0, mh / 2 + 1, mw - 1, mh / 2 + 1, LED_RED_HIGH);

    // 4 levels of crossing green lines.
    matrix->drawLine(mw / 2 - 2, 0, mw / 2 - 2, mh - 1, LED_GREEN_VERYLOW);
    matrix->drawLine(mw / 2 - 1, 0, mw / 2 - 1, mh - 1, LED_GREEN_LOW);
    matrix->drawLine(mw / 2 + 0, 0, mw / 2 + 0, mh - 1, LED_GREEN_MEDIUM);
    matrix->drawLine(mw / 2 + 1, 0, mw / 2 + 1, mh - 1, LED_GREEN_HIGH);

    // Diagonal blue line.
    matrix->drawLine(0, 0, mw - 1, mh - 1, LED_BLUE_HIGH);
    matrix->drawLine(0, mh - 1, mw - 1, 0, LED_ORANGE_MEDIUM);
    matrix->show();
}

void display_boxes() {
    matrix_clear();
    matrix->drawRect(0, 0, mw, mh, LED_BLUE_HIGH);
    matrix->drawRect(1, 1, mw - 2, mh - 2, LED_GREEN_MEDIUM);
    matrix->fillRect(2, 2, mw - 4, mh - 4, LED_RED_HIGH);
    matrix->fillRect(3, 3, mw - 6, mh - 6, LED_ORANGE_MEDIUM);
    matrix->show();
}

void display_circles() {
    matrix_clear();
    matrix->drawCircle(mw / 2, mh / 2, 2, LED_RED_MEDIUM);
    matrix->drawCircle(mw / 2 - 1 - min(mw, mh) / 8, mh / 2 - 1 - min(mw, mh) / 8, min(mw, mh) / 4, LED_BLUE_HIGH);
    matrix->drawCircle(mw / 2 + 1 + min(mw, mh) / 8, mh / 2 + 1 + min(mw, mh) / 8, min(mw, mh) / 4 - 1, LED_ORANGE_MEDIUM);
    matrix->drawCircle(1, mh - 2, 1, LED_GREEN_LOW);
    matrix->drawCircle(mw - 2, 1, 1, LED_GREEN_HIGH);
    if (min(mw, mh) > 12) matrix->drawCircle(mw / 2 - 1, mh / 2 - 1, min(mh / 2 - 1, mw / 2 - 1), LED_CYAN_HIGH);
    matrix->show();
}

void display_textes() {
    matrix_clear();
}

// ---------------------------------------------------------------------------
// Shared functions
// ---------------------------------------------------------------------------

uint16_t Color24toColor16(uint32_t color) {
    return ((uint16_t) (((color & 0xFF0000) >> 16) & 0xF8) << 8) | ((uint16_t) (((color & 0x00FF00) >> 8) & 0xFC) << 3)
            | (((color & 0x0000FF) >> 0) >> 3);
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
    uint32_t wheel = 0;

    // Serial.print(WheelPos);
    WheelPos = 255 - WheelPos;
    if (WheelPos < 85) {
        wheel = (((uint32_t) (255 - WheelPos * 3)) << 16) + (WheelPos * 3);
    }
    if (!wheel && WheelPos < 170) {
        WheelPos -= 85;
        wheel = (((uint32_t) (WheelPos * 3)) << 8) + (255 - WheelPos * 3);
    }
    if (!wheel) {
        WheelPos -= 170;
        wheel = (((uint32_t) (WheelPos * 3)) << 16) + (((uint32_t) (255 - WheelPos * 3)) << 8);
    }
    // Serial.print(" -> ");
    // Serial.println(wheel, HEX);
    return (wheel);
}

// ---------------------------------------------------------------------------
// Matrix Code
// ---------------------------------------------------------------------------

// type 0 = up, type 1 = up and down
// The code is complicated looking, but the state machine is so that
// you can call this function to do a frame update and then switch to
// other work
// There are some magic numbers I had to hand tune for a 24x32 array.
// You'll have to adjust for your own array size
uint8_t font_zoom(uint8_t zoom_type, uint8_t speed) {
    static uint16_t state;
    static uint16_t direction;
    static uint16_t size;
    static uint8_t l;
    static int16_t faster = 0;
    static bool dont_exit;
    static uint16_t delayframe;
    char letters[] = { 'H', 'E', 'L', 'l', 'O' };
    bool done = 0;
    uint8_t repeat = 3;

    if (matrix_reset_demo == 1) {
        matrix_reset_demo = 0;
        state = 1;
        direction = 1;
        //size = 3;
        size = 1;
        l = 0;
        if (matrix_loop == -1) {
            dont_exit = 1;
            delayframe = 2;
            faster = 0;
        };
    }

    matrix->setTextSize(1);

    if (--delayframe) {
        // reset how long a frame is shown before we switch to the next one
        // Serial.println("delayed frame");
        matrix->show(); // make sure we still run at the same speed.
        return repeat;
    }
    delayframe = max((speed / 10) - faster, 1);
    // before exiting, we run the full delay to show the last frame long enough
    if (dont_exit == 0) {
        dont_exit = 1;
        return 0;
    }
    if (direction == 1) {
        int8_t offset = 0; // adjust some letters left or right as needed
        if (letters[l] == 'H') offset = -3 * size / 15;
        if (letters[l] == 'O') offset = -3 * size / 15;
        if (letters[l] == 'l') offset = +5 * size / 15;

        uint16_t txtcolor = Color24toColor16(Wheel(map(letters[l], '0', 'Z', 0, 255)));
        matrix->setTextColor(txtcolor);

        matrix->clear();
        matrix->setFont(&Century_Schoolbook_L_Bold[size]);
        //matrix->setCursor(10 - size * 0.55 + offset, 17 + size * 0.75);
        matrix->setCursor(0 - size * 0.55 + offset, 0 + size * 0.75);
        matrix->print(letters[l]);
        if (size < 18) size++;
        else if (zoom_type == 0) {
            done = 1;
            delayframe = max((speed - faster * 10) * 1, 3);
        }
        else direction = 2;

    }
    else if (zoom_type == 1) {
        int8_t offset = 0; // adjust some letters left or right as needed
        uint16_t txtcolor = Color24toColor16(Wheel(map(letters[l], '0', 'Z', 255, 0)));
        if (letters[l] == 'H') offset = -3 * size / 15;
        if (letters[l] == 'O') offset = -3 * size / 15;
        if (letters[l] == 'l') offset = +5 * size / 15;

        matrix->setTextColor(txtcolor);
        matrix->clear();
        matrix->setFont(&Century_Schoolbook_L_Bold[size]);
        //matrix->setCursor(10 - size * 0.55 + offset, 17 + size * 0.75);
        matrix->setCursor(0 - size * 0.55 + offset, 0 + size * 0.75);
        matrix->print(letters[l]);
        if (size > 3) size--;
        else {
            done = 1;
            direction = 1;
            delayframe = max((speed - faster * 10) / 2, 3);
        };
    }

    matrix->show();
    //Serial.println("done?");
    if (!done) return repeat;
    direction = 1;
    size = 3;
    //Serial.println("more letters?");
    if (++l < sizeof(letters)) return repeat;
    l = 0;
    //Serial.println("reverse pattern?");
    if (zoom_type == 1 && direction == 2) return repeat;

    //Serial.println("Done with font animation");
    faster++;
    matrix_reset_demo = 1;
    dont_exit = 0;
    // Serial.print("delayframe on last letter ");
    // Serial.println(delayframe);
    // After showing the last letter, pause longer
    // unless it's a zoom in zoom out.
    if (zoom_type == 0) delayframe *= 5;
    else delayframe *= 3;
    return repeat;
}
void font_loop() {
    uint8_t ret;
    static uint8_t cnt = 0;

    delay(5);
    // this code is meant to be run at some interval and keep its on
    // state while you do something else.

    if (cnt % 2) {
        ret = font_zoom(1, 25);
    }
    else {
        ret = font_zoom(0, 30);
    }
    if (matrix_loop == -1) matrix_loop = ret;
    if (ret) return;
    Serial.print("Finished run #");
    Serial.println(matrix_loop);
    if (matrix_loop-- > 0) return;
    Serial.println("Animation loop done, switching to other demo");
    cnt++;
    //delay(1000);
    matrix_reset_demo = 1;
}

