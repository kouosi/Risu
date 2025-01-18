#pragma once
#include <Arduino.h>

#ifndef DEFAULT_SPEED
#define DEFAULT_SPEED 255
#endif

class Motor {
public:
    typedef enum {
        FWD,
        BWD
    } Direction;

    Motor(uint8_t pwmPin, uint8_t in1Pin, uint8_t in2Pin, uint8_t stbyPin, int8_t offset);
    void drive(uint8_t speed, Direction dir);
    void drive(uint8_t speed, Direction dir, uint32_t delay);
    void brake();
    void setStandBy();

private:
    uint8_t in1Pin, in2Pin, pwmPin, stbyPin, speed;
    int8_t offset;
    void fwd(uint8_t speed);
    void rev(uint8_t speed);
};

void forward(Motor motor1, Motor motor2, int speed);
void forward(Motor motor1, Motor motor2);

void backward(Motor motor1, Motor motor2, int speed);
void backward(Motor motor1, Motor motor2);

void turnLeft(Motor left, Motor right, int speed);
void turnRight(Motor left, Motor right, int speed);

void brake(Motor motor1, Motor motor2);
void getInfo(Motor motor);
