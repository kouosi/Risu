#include <pins_arduino.h>
#include <Arduino.h>
#include <EEPROM.h>
#include <OneButton.h>
#include <BlockNot.h>
#include "Motor.hpp"
#include "config.hpp"

////////////////////////////////////////////////////////
// Variables
////////////////////////////////////////////////////////
// Our Motors
Motor motor1 = Motor(APWM_PIN, AIN1_PIN, AIN2_PIN, STBY_PIN, 1);
Motor motor2 = Motor(BPWM_PIN, BIN1_PIN, BIN2_PIN, STBY_PIN, 1);

// To blink led
BlockNot ledBlink(500);
boolean ledState = false;

// For our trigger button
OneButton btn(TRIG_PIN);

// PID
int P, D, I, previousError, PIDvalue, error;
int lsp, rsp;
int lfspeed = 200;

float Kp = 0;
float Kd = 0;
float Ki = 0;

// calibration
int minValues[6], maxValues[6], threshold[6];

////////////////////////////////////////////////////////
// Calibration
////////////////////////////////////////////////////////
void calibrate() {
    for (int i = 1; i < 6; i++) {
        minValues[i] = analogRead(i - 1);
        maxValues[i] = analogRead(i - 1);
    }

    for (int i = 0; i < 3000; i++) {
        motor1.drive(50, Motor::FWD);
        motor2.drive(50, Motor::BWD);

        for (int i = 1; i < 6; i++) {
            if (analogRead(i) < minValues[i]) {
                minValues[i] = analogRead(i);
            }
            if (analogRead(i) > maxValues[i]) {
                maxValues[i] = analogRead(i);
            }
        }
    }

    for (int i = 1; i < 6; i++) {
        threshold[i] = (minValues[i] + maxValues[i]) / 2;
        Serial.print(threshold[i]);
        Serial.print("   ");
    }

    Serial.println();

    motor1.drive(0, Motor::FWD);
    motor2.drive(0, Motor::FWD);
}

void calibrateSetup() {
    delay(1000);
    if (ledBlink.triggered()) {
        ledState = !ledState;
        digitalWrite(STATUS_LED, ledState ? HIGH : LOW);
    }
    for (size_t i = 0; i < 3; i++) {
    }
}

////////////////////////////////////////////////////////
// Debugging
////////////////////////////////////////////////////////
void transmitData() {
    // → Motor speed
    // → IR sensor data
    // ← PID value
}

////////////////////////////////////////////////////////
// Normal Run
////////////////////////////////////////////////////////
void linefollow() {
    int error = (analogRead(SENSOR2) - analogRead(SENSOR4));

    P = error;
    I = I + error;
    D = error - previousError;

    PIDvalue = (Kp * P) + (Ki * I) + (Kd * D);
    previousError = error;

    lsp = lfspeed - PIDvalue;
    rsp = lfspeed + PIDvalue;

    if (lsp > 255) {
        lsp = 255;
    }
    if (lsp < 0) {
        lsp = 0;
    }
    if (rsp > 255) {
        rsp = 255;
    }
    if (rsp < 0) {
        rsp = 0;
    }
    motor1.drive(lsp, Motor::FWD);
    motor2.drive(rsp, Motor::FWD);
}

void process() {
    if (analogRead(1) > threshold[1] && analogRead(5) < threshold[5]) {
        lsp = 0;
        rsp = lfspeed;
        motor1.drive(0, Motor::FWD);
        motor2.drive(lfspeed, Motor::FWD);
    }

    else if (analogRead(5) > threshold[5] && analogRead(1) < threshold[1]) {
        lsp = lfspeed;
        rsp = 0;
        motor1.drive(lfspeed, Motor::FWD);
        motor2.drive(0, Motor::FWD);
    } else if (analogRead(3) > threshold[3]) {
        Kp = 0.0006 * (1000 - analogRead(3));
        Kd = 10 * Kp;
        // Ki = 0.0001;
        linefollow();
    }
}

void runSetup() {
    delay(1000);

    digitalWrite(STATUS_LED, HIGH);
    digitalWrite(LED_BUILTIN, LOW);

    while (1) {
        process();
#ifdef DEBUG
        transmitData();
#endif
    }
}

////////////////////////////////////////////////////////
// Setting Saving
////////////////////////////////////////////////////////
typedef struct {
    float kp;
    float ki;
    float kd;
    bool isAvailable;
} PIDSettings;

void savePIDData() {
    PIDSettings pid_values = {Kp, Ki, Kd, true};
    EEPROM.put(0, pid_values);
}

PIDSettings readPIDData() {
    PIDSettings pid;
    EEPROM.get(0, pid);
    return pid;
}

////////////////////////////////////////////////////////
// Arduino
////////////////////////////////////////////////////////
void setup() {
    Serial.begin(9600);
    btn.setup(TRIG_PIN);
    btn.attachClick(runSetup);
    btn.attachDoubleClick(calibrateSetup);
#ifdef DEBUG
    btn.attachLongPressStop(savePIDData);
#endif
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(STATUS_LED, OUTPUT);
}

void loop() {
    btn.tick();
    if (ledBlink.triggered()) {
        ledState = !ledState;
        digitalWrite(LED_BUILTIN, ledState ? HIGH : LOW);
    }
    delay(50);
}
