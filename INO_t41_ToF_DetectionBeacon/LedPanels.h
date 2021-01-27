/*
 * LedPanels.h
 *
 *  Created on: Jan 25, 2021
 *      Author: cho (PM-ROBOTIX)
 */

#ifndef LEDPANELS_H_
#define LEDPANELS_H_

#include <WS2812Serial.h>
#define USE_WS2812SERIAL //doit etre placé après #include <WS2812Serial.h> pour utiliser la librairie compatible teensy4

#include <Adafruit_GFX.h>
#include <FastLED.h>
#include <FastLED_NeoMatrix.h>
#include <LEDMatrix.h>
#include "TeensyThreads.h"

#define DISABLE_WHITE


// Used by LEDMatrix
#define MATRIX_TILE_WIDTH   4 // width of EACH NEOPIXEL MATRIX (not total display)
#define MATRIX_TILE_HEIGHT  4 // height of each matrix
#define MATRIX_TILE_H       2  // number of matrices arranged horizontally
#define MATRIX_TILE_V       9  // number of matrices arranged vertically

// Used by NeoMatrix
#define mw (MATRIX_TILE_WIDTH *  MATRIX_TILE_H)
#define mh (MATRIX_TILE_HEIGHT * MATRIX_TILE_V)
#define NUMMATRIX (mw*mh)

// Compat for some other demos
#define NUM_LEDS NUMMATRIX
#define MATRIX_HEIGHT mh
#define MATRIX_WIDTH mw

#define BRIGHTNESS 2

#define PIN 8 //serial PIN for WS2812


// This could also be defined as matrix->color(255,0,0) but those defines
// are meant to work for adafruit_gfx backends that are lacking color()
#define LED_BLACK             0

#define LED_RED_VERYLOW       (3 <<  11)
#define LED_RED_LOW                     (7 <<  11)
#define LED_RED_MEDIUM                  (15 << 11)
#define LED_RED_HIGH                    (31 << 11)

#define LED_GREEN_VERYLOW     (1 <<  5)
#define LED_GREEN_LOW                   (15 << 5)
#define LED_GREEN_MEDIUM      (31 << 5)
#define LED_GREEN_HIGH                  (63 << 5)

#define LED_BLUE_VERYLOW      3
#define LED_BLUE_LOW                    7
#define LED_BLUE_MEDIUM       15
#define LED_BLUE_HIGH                   31

#define LED_ORANGE_VERYLOW    (LED_RED_VERYLOW + LED_GREEN_VERYLOW)
#define LED_ORANGE_LOW                  (LED_RED_LOW     + LED_GREEN_LOW)
#define LED_ORANGE_MEDIUM     (LED_RED_MEDIUM  + LED_GREEN_MEDIUM)
#define LED_ORANGE_HIGH                 (LED_RED_HIGH    + LED_GREEN_HIGH)

#define LED_PURPLE_VERYLOW    (LED_RED_VERYLOW + LED_BLUE_VERYLOW)
#define LED_PURPLE_LOW                  (LED_RED_LOW     + LED_BLUE_LOW)
#define LED_PURPLE_MEDIUM     (LED_RED_MEDIUM  + LED_BLUE_MEDIUM)
#define LED_PURPLE_HIGH                 (LED_RED_HIGH    + LED_BLUE_HIGH)

#define LED_CYAN_VERYLOW      (LED_GREEN_VERYLOW + LED_BLUE_VERYLOW)
#define LED_CYAN_LOW                    (LED_GREEN_LOW     + LED_BLUE_LOW)
#define LED_CYAN_MEDIUM                 (LED_GREEN_MEDIUM  + LED_BLUE_MEDIUM)
#define LED_CYAN_HIGH                   (LED_GREEN_HIGH    + LED_BLUE_HIGH)

#define LED_WHITE_VERYLOW     (LED_RED_VERYLOW + LED_GREEN_VERYLOW + LED_BLUE_VERYLOW)
#define LED_WHITE_LOW                   (LED_RED_LOW     + LED_GREEN_LOW     + LED_BLUE_LOW)
#define LED_WHITE_MEDIUM      (LED_RED_MEDIUM  + LED_GREEN_MEDIUM  + LED_BLUE_MEDIUM)
#define LED_WHITE_HIGH                  (LED_RED_HIGH    + LED_GREEN_HIGH    + LED_BLUE_HIGH)


void ledPanels_loop();
void ledPanels_setup();
void ledPanels_setup2();

void matrix_clear();

void display_dist();
void sprite_setup();
void flag();
void count_pixels_on_by_one();
void count_pixels();
void display_lines();
void display_boxes();
void display_circles();
void display_textes();

void display_leds();


#endif /* LEDPANELS_H_ */
