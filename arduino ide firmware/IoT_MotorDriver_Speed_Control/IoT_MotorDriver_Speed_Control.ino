/*******************************************************
 * Single-Sketch Example: SparkFun TMAG5273 + SimpleFOC
 * 
 * Demonstrates how to:
 *  - Include the SparkFun TMAG5273 library FIRST
 *  - Define a custom Sensor class referencing SFE_TMAG5273
 *  - Use that sensor in closed-loop angle control
 *******************************************************/

#include <Arduino.h>
#include <Wire.h>
#include <SimpleFOC.h>

// 1) Include SparkFun TMAG5273 libraries BEFORE defining your sensor class
#include <SparkFun_TMAG5273_Arduino_Library.h>
#include <SparkFun_TMAG5273_Arduino_Library_Defs.h>

// ---------------------------------------------------------------------
//  Custom Sensor Class
// ---------------------------------------------------------------------
class SparkFunTMAG5273Sensor : public Sensor {
public:
  // Pass a reference to SFE_TMAG5273
  SparkFunTMAG5273Sensor(SFE_TMAG5273 &tmagObj) : _tmag(tmagObj) {}

  void init() override { _initialized = true; }

  float getAngle() override {
    if (!_initialized) return 0.0f;
    // Example read from XY plane:
    float angleDeg = _tmag.getMagneticAngleXY();
    float angleRad = angleDeg * DEG_TO_RAD;
    // wrap angle in [0..2π)
    angleRad = fmod(angleRad, TWO_PI);
    if (angleRad < 0) angleRad += TWO_PI;
    return angleRad;
  }

  float getVelocity() override { return NOT_SET; }

private:
  SFE_TMAG5273 &_tmag;
  bool _initialized = false;
};

// ---------------------------------------------------------------------
//  Pin definitions
// ---------------------------------------------------------------------
#define UH_PIN 16
#define UL_PIN 17
#define VH_PIN 18
#define VL_PIN 23
#define WH_PIN 19
#define WL_PIN 33

// Current sense pins
#define CUR_SENSE_A 4
#define CUR_SENSE_B 5
#define CUR_SENSE_C 7
#define CUR_SENSE_TOTAL 8

// Buttons
#define BTN_ENABLE 13
#define BTN_SPEED  14

// ---------------------------------------------------------------------
//  Global objects
// ---------------------------------------------------------------------
BLDCMotor motor(7); // e.g. 7 pole pairs
BLDCDriver6PWM driver(UH_PIN, UL_PIN, VH_PIN, VL_PIN, WH_PIN, WL_PIN);
InlineCurrentSense current_sense(0.012f, 3.3f, CUR_SENSE_A, CUR_SENSE_B, CUR_SENSE_C);

SFE_TMAG5273 tmag;  // SparkFun
