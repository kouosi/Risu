#include "Motor.hpp"
#include <Arduino.h>

Motor::Motor(uint8_t pwmPin, uint8_t in1Pin, uint8_t in2Pin, uint8_t stbyPin, int8_t offset) {
    this->in1Pin = in1Pin;
    this->in2Pin = in2Pin;
    this->pwmPin = pwmPin;
    this->stbyPin = stbyPin;
    this->offset = offset;

    pinMode(in1Pin, OUTPUT);
    pinMode(in2Pin, OUTPUT);
    pinMode(pwmPin, OUTPUT);
    pinMode(stbyPin, OUTPUT);
}

void Motor::drive(uint8_t speed, Direction dir) {
    digitalWrite(stbyPin, HIGH);
    speed = speed * offset;

    if (dir == Motor::FWD) {
        fwd(speed);
    } else {
        rev(speed);
    }
}

void Motor::drive(uint8_t speed, Direction dir, uint32_t duration) {
    drive(speed, dir);
    delay(duration);
}

void Motor::fwd(uint8_t speed) {
    digitalWrite(in1Pin, HIGH);
    digitalWrite(in2Pin, LOW);
    analogWrite(pwmPin, speed);
    this->speed = speed;
}

void Motor::rev(uint8_t speed) {
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, HIGH);
    analogWrite(pwmPin, speed);
    this->speed = speed;
}

void Motor::brake() {
    digitalWrite(in1Pin, HIGH);
    digitalWrite(in2Pin, HIGH);
    analogWrite(pwmPin, 0);
}

void Motor::setStandBy() {
    digitalWrite(stbyPin, LOW);
}

void forward(Motor motor1, Motor motor2, uint8_t speed) {
    motor1.drive(speed, Motor::FWD);
    motor2.drive(speed, Motor::FWD);
}

void forward(Motor motor1, Motor motor2) {
    motor1.drive(DEFAULT_SPEED, Motor::FWD);
    motor2.drive(DEFAULT_SPEED, Motor::FWD);
}

void backward(Motor motor1, Motor motor2, uint8_t speed) {
    motor1.drive(speed, Motor::BWD);
    motor2.drive(speed, Motor::BWD);
}

void backward(Motor motor1, Motor motor2) {
    motor1.drive(DEFAULT_SPEED, Motor::BWD);
    motor2.drive(DEFAULT_SPEED, Motor::BWD);
}

void turnLeft(Motor left, Motor right, uint8_t speed) {
    uint8_t half_speed = speed / 2;
    left.drive(half_speed, Motor::BWD);
    right.drive(half_speed, Motor::FWD);
}

void turnRight(Motor left, Motor right, uint8_t speed) {
    uint8_t half_speed = speed / 2;
    left.drive(half_speed, Motor::FWD);
    right.drive(half_speed, Motor::BWD);
}

void brake(Motor motor1, Motor motor2) {
    motor1.brake();
    motor2.brake();
}
