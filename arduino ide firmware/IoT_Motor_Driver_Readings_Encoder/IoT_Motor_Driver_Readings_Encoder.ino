/**************************************************************
  Example: Position Hold (Resistance) with SparkFun IoT Brushless
           Motor Driver (ESP32 + TMC6300), TMAG5273 Hall Sensor,
           SimpleFOC, 2 Buttons, WS2812, and optional TFT display.

  Requirements/Assumptions:
  - We have a TMAG5273 that can give us an approximate angle of a magnet.
  - We do a simple P-control in user code to hold a target angle,
    resisting external rotation.

  Disclaimer:
  - This is a reference design. Adapt pin assignments, sensor reading,
    and motor control details to your specific hardware setup.
**************************************************************/

#include <Arduino.h>
#include <Wire.h>

// SimpleFOC
#include <SimpleFOC.h>  

// TMAG5273 Library (SparkFun)
#include <SparkFun_TMAG5273_Arduino_Library.h>
#include <SparkFun_TMAG5273_Arduino_Library_Defs.h>

// TFT_eSPI (optional for 240×240 GC9A01)
#include <TFT_eSPI.h>

// WS2812 LED
#include <Adafruit_NeoPixel.h>

// ----------------------------------------------------------------------------
// Pin Definitions (UPDATE to your actual wiring!)
// ----------------------------------------------------------------------------
// For TMC6300 6-PWM control (example pins from your working demo):
static const int PWM_AH = 5;
static const int PWM_AL = 6;
static const int PWM_BH = 9;
static const int PWM_BL = 10;
static const int PWM_CH = 3;
static const int PWM_CL = 11;

// Buttons
static const int BUTTON_MODE_PIN  = 13;
static const int BUTTON_VALUE_PIN = 14;

// WS2812 LED
static const int WS2812_PIN      = 2;
static const int WS2812_NUM_LEDS = 1;

// (Optional) TFT SPI pins if needed: 
//   SCK=18, MISO=19, MOSI=23, CS=5
//   => configured in User_Setup.h for TFT_eSPI

// ----------------------------------------------------------------------------
// Global Objects
// ----------------------------------------------------------------------------
BLDCMotor motor(7);  // 7 pole pairs example
BLDCDriver6PWM driver(PWM_AH, PWM_AL, PWM_BH, PWM_BL, PWM_CH, PWM_CL);

TMAG5273 tmag;       // Hall sensor object
TFT_eSPI tft = TFT_eSPI();  
Adafruit_NeoPixel strip(WS2812_NUM_LEDS, WS2812_PIN, NEO_GRB + NEO_KHZ800);

// Commander for debugging or user commands (optional)
Commander command = Commander(Serial);
// Commander callbacks (optional)
void doTarget(char* cmd) { command.scalar(&motor.target, cmd); }
void doLimit(char* cmd)  { command.scalar(&motor.voltage_limit, cmd); }

// ----------------------------------------------------------------------------
// Global Variables
// ----------------------------------------------------------------------------
// For demonstration, we do simple "P-control" around a targetAngle
bool holdEnabled   = true;   // if true, motor holds position
float targetAngle  = 0.0;    // in radians
float measuredAngle= 0.0;    // sensor reading in radians
float pGain        = 10.0;   // proportional gain for "resist"

bool lastButtonModeState  = HIGH;
bool lastButtonValueState = HIGH;

// ----------------------------------------------------------------------------
// 1) Setup Functions
// ----------------------------------------------------------------------------

void setupButtons() {
  pinMode(BUTTON_MODE_PIN,  INPUT_PULLUP);
  pinMode(BUTTON_VALUE_PIN, INPUT_PULLUP);
}

void setupWS2812() {
  strip.begin();
  strip.show(); // all off
}

void setupTFT() {
  tft.init();
  tft.setRotation(0);
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(2);
}

void setupTMAG5273() {
  // Start I2C on pins SDA=21, SCL=22
  Wire.begin(21, 22); 
  delay(100);

  // If your library expects tmag.begin(Wire) or tmag.begin(address, Wire):
  if (!tmag.begin(Wire)) {
    Serial.println("TMAG5273 not detected!");
  } else {
    Serial.println("TMAG5273 detected.");
  }
}

void setupMotor() {
  // Driver config
  driver.voltage_power_supply = 3.3;  // 3.3V logic/power
  driver.voltage_limit        = 3.0;  // limit
  driver.pwm_frequency        = 32000;
  driver.init();
  motor.linkDriver(&driver);

  // Motor config
  motor.voltage_limit = 3.0;  
  // We'll do manual P-control, so set controller to open-loop angle
  motor.controller = MotionControlType::angle_openloop;  
  motor.init();

  // (Optional) Commander setup
  Serial.begin(115200);
  command.add('T', doTarget,  "target angle");
  command.add('L', doLimit,   "voltage limit");
  Serial.println("Motor ready!");
}

// ----------------------------------------------------------------------------
// 2) Loop Functions
// ----------------------------------------------------------------------------

void readButtons() {
  bool modeState  = digitalRead(BUTTON_MODE_PIN);
  bool valueState = digitalRead(BUTTON_VALUE_PIN);

  // If Button_MODE is pressed, shift target angle by 45 degrees
  if (modeState == LOW && lastButtonModeState == HIGH) {
    targetAngle += (PI/4.0);
    if (targetAngle > TWO_PI) {
      targetAngle -= TWO_PI;
    }
    Serial.print("New targetAngle(deg): ");
    Serial.println(targetAngle * 180.0/PI);
  }

  // If Button_VALUE is pressed, toggle hold on/off
  if (valueState == LOW && lastButtonValueState == HIGH) {
    holdEnabled = !holdEnabled;
    Serial.print("HoldEnabled = ");
    Serial.println(holdEnabled ? "TRUE" : "FALSE");
  }

  lastButtonModeState  = modeState;
  lastButtonValueState = valueState;
}

// Example function to read angle from the TMAG5273
void readSensorAngle() {
  // For new SparkFun library:
  if (tmag.checkDataReady()) {
    tmag.readAllAngles(); 
    float angleDeg = tmag.getAngleDeg(); // 0..360
    // Convert degrees -> radians
    measuredAngle = angleDeg * (PI/180.0);
  }

  // If you have an older library, adapt to get X/Y or getAngle() differently.
}

// Simple "P-control" approach to hold the angle
// The motor is in open-loop angle mode, so we manually compute commandedAngle
void updateMotor() {
  if (!holdEnabled) {
    // If hold not enabled, do nothing or set motor to zero angle
    motor.move(0);
    return;
  }

  // 1) Compute angle error
  float angleError = targetAngle - measuredAngle;

  // Keep error in [-PI, +PI] for stability
  if (angleError > PI)  angleError -= TWO_PI;
  if (angleError < -PI) angleError += TWO_PI;

  // 2) Commanded angle = measuredAngle + P * error
  float commandedAngle = measuredAngle + (pGain * angleError);

  // 3) Move the motor to that commanded angle in open-loop
  motor.move(commandedAngle);
}

// Optional function to show info on TFT and/or WS2812
void updateDisplay() {
  // Clear screen
  tft.fillScreen(TFT_BLACK);
  tft.setCursor(0, 0);
  tft.println("Hold Position Demo");
  tft.print("Hold: "); tft.println(holdEnabled ? "ON" : "OFF");

  tft.print("Target(deg): ");
  tft.println(targetAngle * 180.0/PI, 1);

  tft.print("Measured(deg): ");
  tft.println(measuredAngle * 180.0/PI, 1);

  // WS2812 color feedback
  if (holdEnabled) {
    // Green if holding
    strip.setPixelColor(0, strip.Color(0, 255, 0));
  } else {
    // Red if not
    strip.setPixelColor(0, strip.Color(255, 0, 0));
  }
  strip.show();
}

// Optional function to handle serial Commander or debug prints
void updateCommander() {
  command.run();

  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 500) {
    lastPrint = millis();
    Serial.print("HoldEnabled= ");
    Serial.print(holdEnabled);
    Serial.print(" | Target(deg)= ");
    Serial.print(targetAngle * 180.0/PI);
    Serial.print(" | Measured(deg)= ");
    Serial.println(measuredAngle * 180.0/PI);
  }
}

// ----------------------------------------------------------------------------
// Arduino Setup / Loop
// ----------------------------------------------------------------------------

void setup() {
  setupButtons();
  setupWS2812();
  setupTFT();
  setupTMAG5273();
  setupMotor();
}

void loop() {
  // 1) Read user input
  readButtons();

  // 2) Read sensor
  readSensorAngle();

  // 3) Update motor (hold or float)
  updateMotor();

  // 4) Optional: update display / LED
  updateDisplay();

  // 5) Commander / debug
  updateCommander();

  delay(20); // basic refresh period
}
