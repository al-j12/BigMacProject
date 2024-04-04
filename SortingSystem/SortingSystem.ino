#define PRINT_COLOUR  // uncomment to turn on output of colour sensor data

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <MSE2202_Lib.h>
#include <Wire.h>
#include <SPI.h>
#include "Adafruit_TCS34725.h"

// function declarations
void doHeartbeat();  // for mode/heartbeat on Smart LED

// port pin constants
#define MODE_BUTTON 0          // GPIO0  pin 27 for Push Button 1
#define MOTOR_ENABLE_SWITCH 3  // DIP Switch S1-1 pulls Digital pin D3 to ground when on, connected to pin 15 GPIO3 (J3)
#define POT_R1 1               // when DIP Switch S1-3 is on, Analog AD0 (pin 39) GPIO1 is connected to Poteniometer R1
#define SMART_LED 21           // when DIP Switch S1-4 is on, Smart LED is connected to pin 23 GPIO21 (J21)
#define SMART_LED_COUNT 1      // number of SMART LEDs in use
#define BLOCK_SERVO 39         // GPIO43
#define SORTING_SERVO 36       // GPIO36
#define RELEASE_SERVO 45       // GPIO45

// colour sensing constant
#define COLOUR_THRESHOLD 85  // adjust threshold as needed

// constants
const int cDisplayUpdate = 100;  // update interval for Smart LED in milliseconds

const int SortingServoGreen = 2050;   // value for open position of green sorting section
const int SortingServoOther = 1100;   // value for closed position of other colours sorting section
const int BlockServoUp = 1100;        // value for blocking gate fully up
const int BlockServoDown = 2050;      // value for blocking gate fully down
const int ReleaseServoOpen = 1100;    // value for opening back gate
const int ReleaseServoClosed = 2050;  // value for closing back gate

const int cHeartbeatInterval = 75;  // heartbeat update interval, in milliseconds
const int cSmartLED = 21;           // when DIP switch S1-4 is on, SMART LED is connected to GPIO21
const int cSmartLEDCount = 1;       // number of Smart LEDs in use
const int cSDA = 47;                // GPIO pin for I2C data
const int cSCL = 48;                // GPIO pin for I2C clock
const int cTCSLED = 14;             // GPIO pin for LED on TCS34725
const int cLEDSwitch = 46;          // DIP switch S1-2 controls LED on TCS32725

// variables
boolean timeUp3sec = false;     // 3 second timer elapsed flag
boolean timeUp2sec = false;     // 2 second timer elapsed flag
boolean timeUp200msec = false;  // 200 millisecond timer elapsed flag

unsigned int modePBDebounce;          // pushbutton debounce timer count
unsigned long timerCount3sec = 0;     // 3 second timer count in milliseconds
unsigned long timerCount2sec = 0;     // 2 second timer count in milliseconds
unsigned long timerCount200msec = 0;  // 200 millisecond timer count in milliseconds
unsigned long displayTime;            // heartbeat LED update timer
unsigned long previousMicros;         // last microsecond count
unsigned long currentMicros;          // current microsecond count

boolean heartbeatState = true;    // state of heartbeat LED
unsigned long lastHeartbeat = 0;  // time of last heartbeat state change
unsigned long curMillis = 0;      // current time, in milliseconds
unsigned long prevMillis = 0;     // start time for delay cycle, in milliseconds

// Declare SK6812 SMART LED object
//   Argument 1 = Number of LEDs (pixels) in use
//   Argument 2 = ESP32 pin number
//   Argument 3 = Pixel type flags, add together as needed:
//     NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//     NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//     NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//     NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
//     NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)
Adafruit_NeoPixel SmartLEDs(SMART_LED_COUNT, SMART_LED, NEO_RGB + NEO_KHZ800);

// Smart LED brightness for heartbeat
unsigned char LEDBrightnessIndex = 0;
unsigned char LEDBrightnessLevels[] = { 0, 0, 0, 5, 15, 30, 45, 60, 75, 90, 105, 120, 135,
                                        150, 135, 120, 105, 90, 75, 60, 45, 30, 15, 5, 0 };

// TCS34725 colour sensor with 2.4 ms integration time and gain of 4
// see https://github.com/adafruit/Adafruit_TCS34725/blob/master/Adafruit_TCS34725.h for all possible values
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_4X);
bool tcsFlag = 0;
bool colourDetected = 0;
bool greenDetected = 0;
bool otherDetected = 0;

unsigned int robotModeIndex = 0;  // robot operational state
unsigned int modeIndicator[6] = {
  // colours for different modes
  SmartLEDs.Color(255, 0, 0),    // red - stop
  SmartLEDs.Color(0, 255, 0),    // green - run
  SmartLEDs.Color(0, 0, 255),    // blue - empty case
  SmartLEDs.Color(255, 255, 0),  // yellow - empty case
  SmartLEDs.Color(0, 255, 255),  // cyan - empty case
  SmartLEDs.Color(255, 0, 255)   // magenta - empty case
};

// classes defined in MSE2202_Lib
Motion Bot = Motion();  // instance of Motion for motor control

unsigned char predefinedPath;         // state index for ping pong ball finding mode
unsigned long timerCount5sec = 0;     // 5 second timer count in milliseconds
boolean timeUp5sec = false;           // 5 second timer elapsed flag
int turned_amount;                    // to record how much it turned when looking for the ball
boolean timeUp500msec = false;        // 500 millisecond timer elapsed flag
unsigned long timerCount500msec = 0;  // 500 millisecond timer count in milliseconds
int iteration = 0;                    // for smoothing arm raise

void setup() {
  Serial.begin(115200);

  // Set up motors
  Bot.servoBegin("S1", SORTING_SERVO);  // set up sorting servo
  Bot.servoBegin("S2", BLOCK_SERVO);    // set up blocking servo
  Bot.servoBegin("S3", RELEASE_SERVO);  // set up release servo

  // Set up SmartLED
  SmartLEDs.begin();                                     // initialize smart LEDs object
  SmartLEDs.clear();                                     // clear pixel
  SmartLEDs.setPixelColor(0, SmartLEDs.Color(0, 0, 0));  // set pixel colours to black (off)
  SmartLEDs.setBrightness(0);                            // set brightness [0-255]
  SmartLEDs.show();                                      // update LED

  pinMode(MOTOR_ENABLE_SWITCH, INPUT_PULLUP);  // set up motor enable switch with internal pullup
  pinMode(MODE_BUTTON, INPUT_PULLUP);          // Set up mode pushbutton
  modePBDebounce = 0;                          // reset debounce timer count

  Wire.setPins(cSDA, cSCL);           // set I2C pins for TCS34725
  pinMode(cTCSLED, OUTPUT);           // configure GPIO to control LED on TCS34725
  pinMode(cLEDSwitch, INPUT_PULLUP);  // configure GPIO to set state of TCS34725 LED

  // Connect to TCS34725 colour sensor
  if (tcs.begin()) {
    Serial.printf("Found TCS34725 colour sensor\n");
    tcsFlag = true;
  } else {
    Serial.printf("No TCS34725 found ... check your connections\n");
    tcsFlag = false;
  }
}

void loop() {
  uint16_t r, g, b, c;  // RGBC values from TCS34725

  digitalWrite(cTCSLED, !digitalRead(cLEDSwitch));  // turn on onboard LED if switch state is low (on position)
  if (tcsFlag) {                                    // if colour sensor initialized
    tcs.getRawData(&r, &g, &b, &c);                 // get raw RGBC values
#ifdef PRINT_COLOUR
    Serial.printf("R: %d, G: %d, B: %d, C %d\n", r, g, b, c);
#endif

    colourDetected = 0;

    Bot.ToPosition("S2", BlockServoDown);
    Serial.printf("colour is not detected");

    //  check if colour is detected
    if (c < COLOUR_THRESHOLD) {
      colourDetected = 1;
      if (g > r && g > b) {
        greenDetected = 1;
        otherDetected = 0;
      } else {
        greenDetected = 0;
        otherDetected = 1;
      }
    }

    // when a colour is detected:
    if (colourDetected) {
      Serial.printf("colour is detected");
      Bot.ToPosition("S2", BlockServoUp);  // open block servo

      // enable sorting servo based on the colour
      if (greenDetected) {
        Serial.printf("it's green");
        Bot.ToPosition("S1", SortingServoGreen);

        timeUp500msec = false;

        while (!timeUp500msec) {
          currentMicros = micros();                        // get current time in microseconds
          if ((currentMicros - previousMicros) >= 1000) {  // enter when 1 ms has elapsed
            previousMicros = currentMicros;                // record current time in microseconds
            timerCount500msec = timerCount500msec + 1;     // Increment 500 millisecond timer count
            if (timerCount500msec > 500)                   // If 500 milliseconds have elapsed
            {
              timerCount500msec = 0;  // Reset 500 millisecond timer count
              timeUp500msec = true;   // Indicate that 500 milliseconds have elapsed
            }
          }
        }

      } else if (otherDetected) {
        Serial.printf("it's not green");
        Bot.ToPosition("S1", SortingServoOther);
        timeUp500msec = false;

        while (!timeUp500msec) {
          currentMicros = micros();                        // get current time in microseconds
          if ((currentMicros - previousMicros) >= 1000) {  // enter when 1 ms has elapsed
            previousMicros = currentMicros;                // record current time in microseconds
            timerCount500msec = timerCount500msec + 1;     // Increment 500 millisecond timer count
            if (timerCount500msec > 500)                   // If 500 milliseconds have elapsed
            {
              timerCount500msec = 0;  // Reset 500 millisecond timer count
              timeUp500msec = true;   // Indicate that 500 milliseconds have elapsed
            }
          }
        }
      }

      if (greenDetected && otherDetected) {
        Serial.printf("all done");
        Bot.ToPosition("S3", ReleaseServoOpen);  // open the back gate if all colours are detected
      }
    }
  }

  currentMicros = micros();                        // get current time in microseconds
  if ((currentMicros - previousMicros) >= 1000) {  // enter when 1 ms has elapsed
    previousMicros = currentMicros;                // record current time in microseconds

    // 5 second timer, counts 5000 milliseconds
    timerCount5sec = timerCount5sec + 1;  // increment 5 second timer count
    if (timerCount5sec > 5000) {          // if 5 seconds have elapsed
      timerCount5sec = 0;                 // reset 5 second timer count
      timeUp5sec = true;                  // indicate that 5 seconds have elapsed
    }

    // 3 second timer, counts 3000 milliseconds
    timerCount3sec = timerCount3sec + 1;  // increment 3 second timer count
    if (timerCount3sec > 4000) {          // if 3 seconds have elapsed
      timerCount3sec = 0;                 // reset 3 second timer count
      timeUp3sec = true;                  // indicate that 3 seconds have elapsed
    }

    // 2 second timer, counts 2000 milliseconds
    timerCount2sec = timerCount2sec + 1;  // increment 2 second timer count
    if (timerCount2sec > 2000) {          // if 2 seconds have elapsed
      timerCount2sec = 0;                 // reset 2 second timer count
      timeUp2sec = true;                  // indicate that 2 seconds have elapsed
    }

    // 200 millisecond timer, counts 200 milliseconds
    timerCount200msec = timerCount200msec + 1;  // Increment 200 millisecond timer count
    if (timerCount200msec > 200)                // If 200 milliseconds have elapsed
    {
      timerCount200msec = 0;  // Reset 200 millisecond timer count
      timeUp200msec = true;   // Indicate that 200 milliseconds have elapsed
    }

    // 500 millisecond timer, counts 500 milliseconds
    timerCount500msec = timerCount500msec + 1;  // Increment 500 millisecond timer count
    if (timerCount500msec > 500)                // If 500 milliseconds have elapsed
    {
      timerCount500msec = 0;  // Reset 500 millisecond timer count
      timeUp500msec = true;   // Indicate that 500 milliseconds have elapsed
    }

    // Mode pushbutton debounce and toggle
    if (!digitalRead(MODE_BUTTON)) {  // if pushbutton GPIO goes LOW (nominal push)
      // Start debounce
      if (modePBDebounce <= 25) {             // 25 millisecond debounce time
        modePBDebounce = modePBDebounce + 1;  // increment debounce timer count
        if (modePBDebounce > 25) {            // if held for at least 25 mS
          modePBDebounce = 1000;              // change debounce timer count to 1 second
        }
      }
      if (modePBDebounce >= 1000) {  // maintain 1 second timer count until release
        modePBDebounce = 1000;
      }
    } else {                       // pushbutton GPIO goes HIGH (nominal release)
      if (modePBDebounce <= 26) {  // if release occurs within debounce interval
        modePBDebounce = 0;        // reset debounce timer count
      } else {
        modePBDebounce = modePBDebounce + 1;    // increment debounce timer count
        if (modePBDebounce >= 1025) {           // if pushbutton was released for 25 ms
          modePBDebounce = 0;                   // reset debounce timer count
          robotModeIndex++;                     // switch to next mode
          robotModeIndex = robotModeIndex & 7;  // keep mode index between 0 and 7
          timerCount3sec = 0;                   // reset 3 second timer count
          timeUp3sec = false;                   // reset 3 second timer
        }
      }
    }

    // Update brightness of heartbeat display on SmartLED
    displayTime++;                                             // count milliseconds
    if (displayTime > cDisplayUpdate) {                        // when display update period has passed
      displayTime = 0;                                         // reset display counter
      LEDBrightnessIndex++;                                    // shift to next brightness level
      if (LEDBrightnessIndex > sizeof(LEDBrightnessLevels)) {  // if all defined levels have been used
        LEDBrightnessIndex = 0;                                // reset to starting brightness
      }
      SmartLEDs.setBrightness(LEDBrightnessLevels[LEDBrightnessIndex]);  // set brightness of heartbeat LED
                                                                         //Indicator();                                                         // update LED
    }
  }
  doHeartbeat();  // update heartbeat LED
}

// update heartbeat LED
void doHeartbeat() {
  curMillis = millis();  // get the current time in milliseconds
  // check to see if elapsed time matches the heartbeat interval
  if ((curMillis - lastHeartbeat) > cHeartbeatInterval) {
    lastHeartbeat = curMillis;                               // update the heartbeat time for the next update
    LEDBrightnessIndex++;                                    // shift to the next brightness level
    if (LEDBrightnessIndex > sizeof(LEDBrightnessLevels)) {  // if all defined levels have been used
      LEDBrightnessIndex = 0;                                // reset to starting brightness
    }
    SmartLEDs.setBrightness(LEDBrightnessLevels[LEDBrightnessIndex]);  // set brightness of heartbeat LED
    SmartLEDs.setPixelColor(0, SmartLEDs.Color(0, 250, 0));            // set pixel colours to green
    SmartLEDs.show();                                                  // update LED
  }
}