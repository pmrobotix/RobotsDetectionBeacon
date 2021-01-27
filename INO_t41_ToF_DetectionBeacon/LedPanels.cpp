/*
 * LedPanels.cpp
 *
 *  Created on: Jan 25, 2021
 *      Author: cho (PM-ROBOTIX)
 */

#include "LedPanels.h"

// cLEDMatrix defines
cLEDMatrix<-MATRIX_TILE_WIDTH, -MATRIX_TILE_HEIGHT, HORIZONTAL_ZIGZAG_MATRIX,
MATRIX_TILE_H, MATRIX_TILE_V, HORIZONTAL_ZIGZAG_BLOCKS> ledmatrix;

// Normally we would define this:
//CRGB leds[NUMMATRIX];

// cLEDMatrix creates a FastLED array inside its object and we need to retrieve
// a pointer to its first element to act as a regular FastLED array, necessary
// for NeoMatrix and other operations that may work directly on the array like FadeAll.
CRGB *leds = ledmatrix[0];

FastLED_NeoMatrix *matrix = new FastLED_NeoMatrix(leds, MATRIX_TILE_WIDTH, MATRIX_TILE_HEIGHT,
MATRIX_TILE_H, MATRIX_TILE_V,
NEO_MATRIX_TOP + NEO_MATRIX_LEFT +
NEO_MATRIX_ROWS + NEO_MATRIX_ZIGZAG +
NEO_TILE_TOP + NEO_TILE_LEFT + NEO_TILE_ROWS + NEO_TILE_PROGRESSIVE);

void matrix_clear() {
    // FastLED.clear does not work properly with multiple matrices connected via parallel inputs
    // on ESP8266 (not sure about other chips).
    memset(leds, 0, NUMMATRIX * 3);
}

void ledPanels_loop() {

    threads.yield();

}

void ledPanels_setup() {

    // Normal output
    FastLED.addLeds<WS2812SERIAL, PIN, GRB>(leds, NUMMATRIX).setCorrection(TypicalLEDStrip);

    // Time for serial port to work?
    //delay(1000);
    //Serial.begin(115200);
    Serial.print("Matrix Size: ");
    Serial.print(mw);
    Serial.print(" ");
    Serial.println(mh);
    matrix->begin();
    matrix->setTextWrap(false);
    matrix->setBrightness(BRIGHTNESS);
    // Mix in an init of LEDMatrix
    sprite_setup();
    delay(1000);
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
    delay(3000);
    matrix_clear();
    Serial.println("First matrix_clear done");
#endif

    matrix_clear();
    matrix->show();

    threads.addThread(display_leds);
}

void display_leds() {
    int t=0;
    while (1) {
        //TEST LEDS
        t++;
        //Serial.println("Count pixels");
        //count_pixels();
        //Serial.println("Count pixels done");
        //delay(400);
        //threads.yield();
        //count_pixels_on_by_one();
        //flag();
        //display_boxes();
        display_dist();
        threads.delay(500);
        //sprite_setup();
        //display_lines();
        threads.delay(500);
        Serial.println (t);


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
    }
}

void display_dist(){
    matrix_clear();
    //ledmatrix.DrawLine(0, 0, ledmatrix.Width() - 1, ledmatrix.Height() - 1, CRGB(0, 255, 255));
    //matrix->drawLine(0, 0, mw - 1, mh - 1, LED_BLUE_HIGH);
    matrix->drawLine(0, mh - 1, mw - 1, 0, CRGB(0, 255, 255));

    //matrix->drawCircle(mw / 2, mh / 2, 2, LED_RED_MEDIUM);
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

