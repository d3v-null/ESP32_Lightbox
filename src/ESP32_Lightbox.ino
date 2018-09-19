/**************************************************************************/
/*!
    @file     ESP32_Lightbox.ino
    @author   Derwent McElhinney

    A DMX master for controlling lighting that can be re-programmed over wifi
    and controlled over bluetooth
 */
/**************************************************************************/
#include <LXESP32DMX.h>
#include "esp_task_wdt.h"
// #include <WiFi.h>
// #include <ESPmDNS.h>
// #include <WiFiUdp.h>
// #include <ArduinoOTA.h>

/**
 * DMX Stuff
 */
uint8_t dmxbuffer[DMX_MAX_FRAME];

#define DMX_DIRECTION_PIN 21
#define DMX_SERIAL_OUTPUT_PIN 17

#define FIRST_DMX_ADDR 1

void copyDMXToOutput(void) {
        xSemaphoreTake( ESP32DMX.lxDataLock, portMAX_DELAY );
        for (int i=1; i<DMX_MAX_FRAME; i++) {
                ESP32DMX.setSlot(i, dmxbuffer[i]);
        }
        xSemaphoreGive( ESP32DMX.lxDataLock );
}

/**
 * Animation stuff
 */

// All possible animation modes
enum animation {
        SOLID,
        RAINBOW,
        FIRE
};
// The total number of frames in looping animations
#define MAX_FRAME 255
#define NUMBER_OF_STRIPS 5
// number of ticks (15ms) to wait between animation frames
#define ANIM_SPEED 50
// Brightness used by some animation modes
#define BRIGHTNESS 200
// Saturation used by some animation modes
#define SATURATION 255
// in fire mode, temperatures above this range cool exponentially
#define FIRE_TEMP 80
// in fire mode, how noisy the fire is when below FIRE_TEMP
#define FIRE_NOISE (FIRE_TEMP / 10)
// in fire mode, how quickly the fire decays
#define FIRE_DECAY 1.15
// in fire mode, Minimum amount a jump will increase the temperature by
#define FIRE_FLARE_JUMP 75

// temporary storage of HSV colour space value for animations
uint8_t hsv[3];
// temporary storage of RGB colour space value for animations
uint8_t rgb[3];
// temperatures of each strip
uint8_t *temperatures;
// Average temperature
uint8_t average_temp;

uint8_t animation;  // the current animation number
uint8_t frame;      // the current frame of the animation


void hsv2rgb(){
        // populate the global RGB storage from the global HSV storage
        // stolen from https://github.com/ratkins/RGBConverter/blob/master/RGBConverter.cpp
        double h = hsv[0] / 255.0;
        double s = hsv[1] / 255.0;
        double v = hsv[2] / 255.0;
        double r = 0.0;
        double g = 0.0;
        double b = 0.0;

        int i = int(h * 6);
        double f = h * 6 - i;
        double p = v * (1 - s);
        double q = v * (1 - f * s);
        double t = v * (1 - (1 - f) * s);

        switch(i % 6) {
        case 0: r = v, g = t, b = p; break;
        case 1: r = q, g = v, b = p; break;
        case 2: r = p, g = v, b = t; break;
        case 3: r = p, g = q, b = v; break;
        case 4: r = t, g = p, b = v; break;
        case 5: r = v, g = p, b = q; break;
        }

        rgb[0] = uint8_t(r * 255);
        rgb[1] = uint8_t(g * 255);
        rgb[2] = uint8_t(b * 255);
}

void temp2RGB(uint8_t temperature) {
        // stolen from https://github.com/techinc/lewd/blob/master/animations/fire.py
        float x = temperature / 255.0;
        float r = pow(x, 1.0) * 3.0;
        float g = pow(x, 1.5) * 2.0;
        float b = pow(x, 3.0);
        if (r > 1.0) r = 1.0;
        if (g > 1.0) g = 1.0;
        if (b > 1.0) b = 1.0;
        rgb[0] = (uint8_t)(r*BRIGHTNESS);
        rgb[1] = (uint8_t)(g*BRIGHTNESS);
        rgb[2] = (uint8_t)(b*BRIGHTNESS);
}

void gammacorrectRGB(){
        // TODO: gamme correct the values stored in the global RBB storage
}


/**
 * Wifi Stuff
 */

void setup() {
        Serial.begin(115200);
        Serial.print("setup");

        pinMode(DMX_DIRECTION_PIN, OUTPUT);
        digitalWrite(DMX_DIRECTION_PIN, HIGH);

        pinMode(DMX_SERIAL_OUTPUT_PIN, OUTPUT);
        ESP32DMX.startOutput(DMX_SERIAL_OUTPUT_PIN);
        Serial.println("setup complete");

        frame = 0;
        animation = FIRE;

        temperatures = (uint8_t*) malloc( NUMBER_OF_STRIPS * sizeof(uint8_t) );

        for (int s = 0; s<NUMBER_OF_STRIPS; s++) {
                temperatures[s] = FIRE_TEMP;
        }
        rgb[0] = BRIGHTNESS; rgb[1] = BRIGHTNESS; rgb[2] = BRIGHTNESS;
}

void loop() {
        frame = (frame + 1) % MAX_FRAME;

        switch( animation ) {
        case RAINBOW:
                hsv[1] = SATURATION;
                hsv[2] = BRIGHTNESS;
                for (int s = 0; s<NUMBER_OF_STRIPS; s++) {
                        // set hue
                        hsv[0] = (frame + (s * MAX_FRAME / NUMBER_OF_STRIPS)) % MAX_FRAME;
                        hsv2rgb();
                        gammacorrectRGB();
                        dmxbuffer[FIRST_DMX_ADDR + (s * 3) + 0] = rgb[1];
                        dmxbuffer[FIRST_DMX_ADDR + (s * 3) + 1] = rgb[0];
                        dmxbuffer[FIRST_DMX_ADDR + (s * 3) + 2] = rgb[2];
                }
                break;
        case FIRE:
                for (int s = 0; s<NUMBER_OF_STRIPS; s++) {
                        // only a 50% chance that the strip will be updated (adds randomness)
                        if (random(100) < 50) continue;
                        // recalculate the approximate average temperature
                        // (I know there's a hack way of doing this but this is more readable)
                        average_temp = 0;
                        for (int s = 0; s<NUMBER_OF_STRIPS; s++) {
                                Serial.print(temperatures[s]);
                                Serial.print(' ');
                                average_temp += temperatures[s] / NUMBER_OF_STRIPS;
                        }
                        Serial.println(average_temp);

                        if (random(100) < 10 && average_temp < 128) {
                                // 10% chance of fire "flaring" if the fire is not hot enough
                                temperatures[s] = random(FIRE_FLARE_JUMP, _min(FIRE_FLARE_JUMP* 2, 255));
                        } else {
                                // otherwise business as usual

                                // if fire is hot, then quickly cool it down, exponentially
                                if (temperatures[s] > FIRE_TEMP) {
                                        temperatures[s] = (uint8_t) temperatures[s] / (FIRE_DECAY);
                                } else {
                                        // 50-50 chance it will go up or down by up to FIRE_NOISE
                                        if (random(2) < 1) {
                                                temperatures[s] += random(FIRE_NOISE);
                                        } else {
                                                // I know this could underflow, that's intended.
                                                temperatures[s] -= random(FIRE_NOISE);
                                        }
                                }
                        }
                        temp2RGB(temperatures[s]);
                        dmxbuffer[FIRST_DMX_ADDR + (s * 3) + 0] = rgb[1];
                        dmxbuffer[FIRST_DMX_ADDR + (s * 3) + 1] = rgb[0];
                        dmxbuffer[FIRST_DMX_ADDR + (s * 3) + 2] = rgb[2];
                }
                break;
        case SOLID:
        default:
                for (int s = 0; s<NUMBER_OF_STRIPS; s++) {
                        dmxbuffer[FIRST_DMX_ADDR + (s * 3) + 0] = rgb[1];
                        dmxbuffer[FIRST_DMX_ADDR + (s * 3) + 1] = rgb[0];
                        dmxbuffer[FIRST_DMX_ADDR + (s * 3) + 2] = rgb[2];
                }
                break;
        };

        copyDMXToOutput();
        delayMicroseconds(ANIM_SPEED * 1000);
}
