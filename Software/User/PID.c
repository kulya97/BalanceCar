//
// Created by huang on 2022-08-13.
//

#include "PID.h"

/**
 * 速度PI控制 修改前进后退速度
 * @param pid pid参数指针
 * @param encoder 左轮编码器、右轮编码器
 * @return 速度控制PWM
 */
float Velocity(Speed_PID *pid, int encoder) {
    static float Velocity = 0, Encoder_Least = 0, Encoder = 0;
    static float Encoder_Integral = 0;
    float a = 0.8;
    float Velocity_Kp = pid->kp, Velocity_Ki = Velocity_Kp / 200.0;
    //=============速度PI控制器=======================//
    Encoder_Least = encoder-pid->target; //===获取最新速度偏差==测量速度（左右编码器之和）-目标速度（此处为零）
    Encoder *= a;                                                  //===一阶低通滤波器
    Encoder += Encoder_Least * (1 - a);                             //===一阶低通滤波器
    Encoder_Integral += Encoder;                             //===积分出位移 积分时间：5ms
    Encoder_Integral = Encoder_Integral;          //===接收遥控器数据，控制前进后退
    if (Encoder_Integral > 10000)
        Encoder_Integral = 10000;              //===积分限幅
    if (Encoder_Integral < -10000)
        Encoder_Integral = -10000;             //===积分限幅
    Velocity = Encoder * Velocity_Kp + Encoder_Integral * Velocity_Ki; //===速度控制
    pid->out = Velocity;
    return Velocity;
}

/**
* 直立PD控制
* @param pid pid参数指针
* @param Angle 角度
* @param Gyro 角速度
* @return 直立控制PWM
*/
float balance(Upright_PID *pid, float Angle, float Gyro) {
    pid->angle = Angle;
    pid->gyro = Gyro;
    float balance = pid->kp * (Angle - pid->target)
                    + pid->kd * (Gyro); //===计算平衡控制的电机PWM  PD控制   kp是P系数 kd是D系数
    pid->out = balance;
    return balance;
}
