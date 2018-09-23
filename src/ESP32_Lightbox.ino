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
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
// #include "BluetoothSerial.h"

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
uint8_t brightness;
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
    rgb[0] = (uint8_t)(r*brightness);
    rgb[1] = (uint8_t)(g*brightness);
    rgb[2] = (uint8_t)(b*brightness);
}

void gammacorrectRGB(){
    // TODO: gamme correct the values stored in the global RBB storage
}


/**
* Wifi Stuff
*/

#define ENABLE_OTA 1



// number of millis between checking
#define OTA_PERIOD 470
unsigned long last_ota = 0;

void setup_ota() {
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    while (WiFi.waitForConnectResult() != WL_CONNECTED) {
        Serial.println("Connection Failed! Rebooting...");
        delay(5000);
        ESP.restart();
    }

    ArduinoOTA
      .onStart([]() {
        String type;
        if (ArduinoOTA.getCommand() == U_FLASH)
          type = "sketch";
        else // U_SPIFFS
          type = "filesystem";

        // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
        Serial.println("Start updating " + type);
      })
      .onEnd([]() {
        Serial.println("\nEnd");
      })
      .onProgress([](unsigned int progress, unsigned int total) {
        Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
      })
      .onError([](ota_error_t error) {
        Serial.printf("Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
        else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
        else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
        else if (error == OTA_END_ERROR) Serial.println("End Failed");
      });

    ArduinoOTA.begin();

    Serial.println("Ready");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
}

/**
 * BLE Stuff
 */

#define ENABLE_BLE 1

#define BLE_PERIOD 530
unsigned long last_ble = 0;

// BluetoothSerial SerialBT;

BLEServer *pServer = NULL;
BLECharacteristic * pTxCharacteristic;
bool deviceConnected = false;
bool oldDeviceConnected = false;
uint8_t txValue = 0;

#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // UART service UUID
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string rxValue = pCharacteristic->getValue();

      if (rxValue.length() > 0) {
        Serial.println("*********");
        Serial.print("Received Value: ");
        for (int i = 0; i < rxValue.length(); i++)
          Serial.print(rxValue[i]);

        Serial.println();
        Serial.println("*********");

        if (rxValue.length() >= 5 && rxValue.substr(0,2) == "!B") {
            // Button press

            // button 1
            // 	!B11:
            // 	!B10;
            // button 2
            // 	!B219
            // 	!B20:
            // Button 3
            // 	!B318
            // 	!B309
            // Button 4
            // 	!B417
            // 	!B408
            // Up
            // 	!B516
            // 	!B507
            // Right
            // 	!B813
            // 	!B804
            // Down
            // 	!B615
            // 	!B606
            // Left
            // 	!B714
            // 	!B705

            if (rxValue.substr(2,5) == "11:") {
                // Serial.println("Button press 1");
                animation = 0;
            }
            if (rxValue.substr(2,5) == "219") {
                // Serial.println("Button press 2");
                animation = 1;
            }
            if (rxValue.substr(2,5) == "318") {
                // Serial.println("Button press 3");
                animation = 2;
            }
            if (rxValue.substr(2,5) == "417") {
                // Serial.println("Button press 4");
                animation = 3;
            }
            if (rxValue.substr(2,5) == "516") {
                // Serial.println("Button press up");
                brightness = (brightness + 16) % 256;
            }
            if (rxValue.substr(2,5) == "813") {
                // Serial.println("Button press right");
            }
            if (rxValue.substr(2,5) == "615") {
                // Serial.println("Button press down");
                brightness = (brightness - 16) % 256;
            }
            if (rxValue.substr(2,5) == "714") {
                // Serial.println("Button press left");
            }
        }

      }
    }
};

/**
 *  Setup
 */

void setup() {
    Serial.begin(115200);
    Serial.print("setup");

    // DMX

    pinMode(DMX_DIRECTION_PIN, OUTPUT);
    digitalWrite(DMX_DIRECTION_PIN, HIGH);

    pinMode(DMX_SERIAL_OUTPUT_PIN, OUTPUT);
    ESP32DMX.startOutput(DMX_SERIAL_OUTPUT_PIN);

    // Animation

    frame = 0;
    animation = FIRE;
    brightness = 100;

    temperatures = (uint8_t*) malloc( NUMBER_OF_STRIPS * sizeof(uint8_t) );

    for (int s = 0; s<NUMBER_OF_STRIPS; s++) {
        temperatures[s] = FIRE_TEMP;
    }
    rgb[0] = brightness; rgb[1] = brightness; rgb[2] = brightness;

    // BLE

    #if ENABLE_BLE

    Serial.println("Starting BLE setup!");

    // if(!SerialBT.begin("ESP32-UART")){
    //     Serial.println("An error occurred initializing Bluetooth");
    // }

    BLEDevice::init("ESP32 UART Service");

    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    // Create the BLE Service
    BLEService *pService = pServer->createService(SERVICE_UUID);

    // Create a BLE Characteristic
    pTxCharacteristic = pService->createCharacteristic(
  										CHARACTERISTIC_UUID_TX,
  										BLECharacteristic::PROPERTY_NOTIFY
  									);

    pTxCharacteristic->addDescriptor(new BLE2902());

    BLECharacteristic * pRxCharacteristic = pService->createCharacteristic(
  											 CHARACTERISTIC_UUID_RX,
  											BLECharacteristic::PROPERTY_WRITE
  										);

    pRxCharacteristic->setCallbacks(new MyCallbacks());
    pService->start();
    pServer->getAdvertising()->addServiceUUID(pService->getUUID());
    pServer->getAdvertising()->start();
    Serial.println("Characteristic defined! Now you can read it in your phone!");

    #endif

    // WIFI

    #if ENABLE_OTA

    setup_ota();

    #endif

}

/**
 *  Loop
 */

void loop() {

    #if ENABLE_OTA

    if (millis() - last_ota > OTA_PERIOD) {
        last_ota = millis();
        // Serial.println("handling OTA");
        ArduinoOTA.handle();
    }

    #endif

    #if ENABLE_BLE

    if (millis() - last_ble > BLE_PERIOD) {
        last_ble = millis();

        // Serial.println("handling BLE");

        // while(SerialBT.available()){
        //     Serial.write(SerialBT.read());
        // }

        // if (deviceConnected) {
        //     pTxCharacteristic->setValue(&txValue, 1);
        //     pTxCharacteristic->notify();
        //     txValue++;
    	// }

        // disconnecting
        if (!deviceConnected && oldDeviceConnected) {
            delay(500); // give the bluetooth stack the chance to get things ready
            pServer->startAdvertising(); // restart advertising
            Serial.println("start advertising");
            oldDeviceConnected = deviceConnected;
        }
        // connecting
        if (deviceConnected && !oldDeviceConnected) {
    		// do stuff here on connecting
            oldDeviceConnected = deviceConnected;
        }
    }

    #endif

    // return;

    frame = (frame + 1) % MAX_FRAME;

    switch( animation ) {
        case RAINBOW:
        hsv[1] = SATURATION;
        hsv[2] = brightness;
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
                // Serial.print(temperatures[s]);
                // Serial.print(' ');
                average_temp += temperatures[s] / NUMBER_OF_STRIPS;
            }
            // Serial.println(average_temp);

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
    delay(ANIM_SPEED);

    // Serial.println("loop end");
}
