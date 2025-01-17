#include <pins_arduino.h>
#include <Arduino.h>
#include <avr/eeprom.h>

#define TRIG_PIN 12
#define STATUS_LED 2

#define PWMA_PIN 3
#define AIN2_PIN 4
#define AIN1_PIN 5
#define STBY_PIN 6
#define BIN1_PIN 7
#define BIN2_PIN 8
#define PWMB_PIN 9

#define SENSOR1 A0
#define SENSOR2 A1
#define SENSOR3 A2
#define SENSOR4 A3
#define SENSOR5 A4


void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(2, OUTPUT);
    Serial.begin(9600);
}

void loop() {
    Serial.println("hi");
    digitalWrite(LED_BUILTIN, HIGH);
    digitalWrite(2, HIGH);
    delay(1000);
    digitalWrite(LED_BUILTIN, LOW);
    digitalWrite(2, LOW);
    delay(1000);
}
