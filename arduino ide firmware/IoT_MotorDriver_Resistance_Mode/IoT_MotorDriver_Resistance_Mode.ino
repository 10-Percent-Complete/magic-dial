#include <Wire.h>
#include <SimpleFOC.h> // http://librarymanager/All#Simple%20FOC

// GPIO
#define auxBtn2 13
#define auxBtn1 14

// Driver
#define uh16 16
#define ul17 17
#define vh18 18
#define wh19 19
#define vl23 23
#define wl33 33

// Current Sense Pins
#define curSense1 4
#define curSense2 5
#define curSense3 7

// Motor driver
BLDCMotor motor = BLDCMotor(7); // 7 pole pairs
BLDCDriver6PWM driver = BLDCDriver6PWM(uh16, ul17, vh18, vl23, wh19, wl33);

// Current sense (shunt resistors: 0.012 ohm)
InlineCurrentSense current_sense = InlineCurrentSense(0.012, 3.3, curSense1, curSense2, curSense3);
float target_torque = 0.0;
bool motor_enabled = false;
float oscillation_frequency = 440.0; // Frequency of oscillation in Hz (e.g., 440 Hz for "A4" pitch)

// Button state
struct Button {
    const uint8_t PIN;
    uint32_t numberKeyPresses;
    bool pressed;
};
Button aux1 = {auxBtn1, 0, false};
Button aux2 = {auxBtn2, 0, false};

// ISR for Button 1 (Adjust frequency)
void IRAM_ATTR isr1() {
    aux1.pressed = true;
    oscillation_frequency += 50; // Increase frequency by 50 Hz
    if (oscillation_frequency > 1000) oscillation_frequency = 100; // Reset frequency if too high
    Serial.print("New oscillation frequency: ");
    Serial.println(oscillation_frequency);
}

// ISR for Button 2 (Enable/Disable motor)
void IRAM_ATTR isr2() {
    aux2.numberKeyPresses++;
    aux2.pressed = true;
    if ((aux2.numberKeyPresses % 2) == 0) {
        motor.disable();
        motor_enabled = false;
        Serial.println("Motor disabled.");
    } else {
        motor.enable();
        motor_enabled = true;
        Serial.println("Motor enabled.");
    }
}

void setup() {
    Serial.begin(115200);

    // Initialize motor driver
    driver.voltage_power_supply = 12; // Set to match your power supply
    driver.init();
    motor.linkDriver(&driver);

    // Initialize current sensing
    current_sense.init();
    motor.linkCurrentSense(&current_sense);

    // Set motor control to torque mode
    motor.controller = MotionControlType::torque;
    motor.voltage_limit = 6.0; // Set safe voltage limit
    motor.init();
    motor.disable();

    // Setup buttons
    pinMode(aux1.PIN, INPUT_PULLUP);
    attachInterrupt(aux1.PIN, isr1, FALLING);
    pinMode(aux2.PIN, INPUT_PULLUP);
    attachInterrupt(aux2.PIN, isr2, FALLING);

    Serial.println("Setup complete. Use aux1 to adjust frequency and aux2 to enable/disable the motor.");
}

void loop() {
    // Handle button presses
    if (aux1.pressed) {
        aux1.pressed = false;
    }
    if (aux2.pressed) {
        aux2.pressed = false;
    }

    // Generate audible noise if motor is enabled
    if (motor_enabled) {
        static unsigned long last_oscillation = 0;
        unsigned long now = millis();
        float oscillation_period = 1000.0 / oscillation_frequency; // Period in ms

        if (now - last_oscillation >= oscillation_period / 2) {
            // Toggle torque direction to create oscillation
            target_torque = -target_torque;
            motor.move(target_torque);
            last_oscillation = now;
        }
    } else {
        motor.move(0.0); // Ensure motor does not move when disabled
    }

    delay(5); // Small delay to stabilize loop
}
