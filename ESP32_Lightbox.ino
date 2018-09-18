/**************************************************************************/
/*!
    @file     outputTest.ino
    @author   Claude Heintz
    @license  BSD (see LXESP32DMX LICENSE)
    @copyright 2017 by Claude Heintz

    Simple Fade test of ESP32 DMX Driver
    @section  HISTORY

    v1.00 - First release
*/
/**************************************************************************/
#include <LXESP32DMX.h>
#include "esp_task_wdt.h"

#define DMX_DIRECTION_PIN 21
#define DMX_SERIAL_OUTPUT_PIN 17

#define FIRST_DMX_ADDR 1

/* Animation stuff */
enum animation {
    RAINBOW,
    FIRE
};
#define MAX_FRAME 255
#define NUMBER_OF_STRIPS 5
// breaks between animation frames
#define ANIM_SPEED 50

#define BRIGHTNESS 50
// full saturation
#define SATURATION 255

uint8_t level;
uint8_t dmxbuffer[DMX_MAX_FRAME];

// temporary storage of HSV colour space value for animations
uint8_t hsv[3];
// temporary storage of RGB colour space value for animations
uint8_t rgb[3];

uint8_t animation;  // the current animation number
uint8_t frame;      // the current frame of the animation

void setup() {
  Serial.begin(115200);
  Serial.print("setup");

  pinMode(DMX_DIRECTION_PIN, OUTPUT);
  digitalWrite(DMX_DIRECTION_PIN, HIGH);

  pinMode(DMX_SERIAL_OUTPUT_PIN, OUTPUT);
  ESP32DMX.startOutput(DMX_SERIAL_OUTPUT_PIN);
  Serial.println("setup complete");

  frame = 0;
  animation = 1;
}

void copyDMXToOutput(void) {
  xSemaphoreTake( ESP32DMX.lxDataLock, portMAX_DELAY );
	for (int i=1; i<DMX_MAX_FRAME; i++) {
    	ESP32DMX.setSlot(i , dmxbuffer[i]);
   }
   xSemaphoreGive( ESP32DMX.lxDataLock );
}

/************************************************************************

  The main loop fades the levels of addresses 1,7,8,510,511, and 512 from zero->full

*************************************************************************/

void hsv2rgb(){
    // populate the global RGB storage from the global HSV storage
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

    switch(i % 6){
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

void gammacorrectRGB(){
    // TODO: gamme correct the values stored in the global RBB storage
}

void loop() {
    frame = (frame + 1) % MAX_FRAME;

    switch( animation ) {
    case RAINBOW:
        hsv[1] = SATURATION;
        hsv[2] = BRIGHTNESS;
        for (int s = 0; s<NUMBER_OF_STRIPS; s++){
            // set hue
            hsv[0] = (frame + (s * MAX_FRAME / NUMBER_OF_STRIPS)) % MAX_FRAME ;
            hsv2rgb();
            dmxbuffer[FIRST_DMX_ADDR + (s * 3) + 0] = rgb[1] ;
            dmxbuffer[FIRST_DMX_ADDR + (s * 3) + 1] = rgb[0] ;
            dmxbuffer[FIRST_DMX_ADDR + (s * 3) + 2] = rgb[2] ;
        }
    case FIRE:
        hsv[1] = SATURATION;
        hsv[2] = BRIGHTNESS;
        for (int s = 0; s<NUMBER_OF_STRIPS; s++){
            // only a 20% chance that the strip will be updated
            if (random(20) < 5) continue;
            hsv[0] = random(255) / 6;
            hsv2rgb();
            dmxbuffer[FIRST_DMX_ADDR + (s * 3) + 0] = rgb[1] ;
            dmxbuffer[FIRST_DMX_ADDR + (s * 3) + 1] = rgb[0] ;
            dmxbuffer[FIRST_DMX_ADDR + (s * 3) + 2] = rgb[2] ;
        }
    };

    copyDMXToOutput();
    esp_task_wdt_feed();
    vTaskDelay(ANIM_SPEED);
}
