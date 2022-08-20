//
// Created by huang on 2022-08-13.
//

#ifndef _PID_H
#define _PID_H

#include "stdint.h"

typedef struct {
    float angle;
    float gyro;
    float kp;
    float ki;
    float kd;
    float target;
    float out;
} Upright_PID;
typedef struct {
    float speed_left;
    float speed_right;
    float kp;
    float ki;
    float kd;
    float target;
    float out;
} Speed_PID;

float Velocity(Speed_PID *pid, int encoder);

float balance(Upright_PID *pid, float Angle, float Gyro);

#endif //DEMO_PID_H
