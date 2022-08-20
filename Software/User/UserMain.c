#include <mpu6050.h>
#include "UserMain.h"
#include "inv_mpu.h"
#include "stdlib.h"
#include <stdio.h>
#include "PID.h"

/**************************************************************************/
TIM_HandleTypeDef *encoder_left_tim = &htim3;
TIM_HandleTypeDef *encoder_right_tim = &htim4;
TIM_HandleTypeDef *encoder_count_tim = &htim2;
TIM_HandleTypeDef *pwm_tim = &htim1;
UART_HandleTypeDef *debug_uart = &huart2;
I2C_HandleTypeDef *mpu6050_i2c = &hi2c1;
//#ifdef __GNUC__
//#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
//#else
//#define PUTCHAR_PROTOTYPE int fputc(int ch. FILE *f)
//#endif /* __GNUC__ */
////重定向printf函数
//PUTCHAR_PROTOTYPE {
//    HAL_UART_Transmit(debug_uart, (uint8_t *) &ch, 1, 0xFFFF);//输出指向串口USART1
//    return ch;
//}
uint8_t USART_TX_BUF[200]; //发送缓冲,最大200字节，不能太小，如果你的内容太长会访问非法内存
#define pf(...)  HAL_UART_Transmit(debug_uart,USART_TX_BUF,sprintf((char *)USART_TX_BUF,__VA_ARGS__),1000)//可修改到其他串口

/**************************************************************************/
uint8_t recv_buffer[32];
uint8_t flag = 0;
/**************************************************************************/
float pitch, roll, yaw;        //欧拉角
short aacx, aacy, aacz;        //加速度传感器原始数据
short gyrox, gyroy, gyroz;    //陀螺仪原始数据
short temp;                    //温度
float speed_left;
float speed_right;

Upright_PID upright_pid;
Speed_PID speed_pid;

/**
 *电机控制
 * @param pwm_left 大于零正转，小于零反转，等于零停止
 * @param pwm_right 大于零正转，小于零反转，等于零停止
 */
void MotorControl(int16_t pwm_left, int16_t pwm_right) {
    if (pwm_left <= 0) {
        HAL_GPIO_WritePin(GPIOB, ain1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, ain2_Pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(GPIOB, ain1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, ain2_Pin, GPIO_PIN_RESET);
    }
    if (pwm_right <= 0) {
        HAL_GPIO_WritePin(GPIOB, bin1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, bin2_Pin, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(GPIOB, bin1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, bin2_Pin, GPIO_PIN_SET);
    }
    __HAL_TIM_SET_COMPARE(pwm_tim, pwm_left_channel, abs(pwm_left));
    __HAL_TIM_SET_COMPARE(pwm_tim, pwm_right_channel, abs(pwm_right));
}

/*
 * 配置参数初始化
 */
void UserConfigInit() {
    upright_pid.kp = 0, upright_pid.kd = 0;
    upright_pid.target = 6;
    speed_pid.kp = 0, speed_pid.ki = 0;
    speed_pid.target = 0;
}

/*
 * 系统初始化
 */
void UserSysInit() {
    /*******************************************/
    HAL_UARTEx_ReceiveToIdle_IT(debug_uart, recv_buffer, sizeof(recv_buffer));//串口空闲中断

    HAL_GPIO_WritePin(GPIOB, ain1_Pin, GPIO_PIN_RESET);//电机方向控制
    HAL_GPIO_WritePin(GPIOB, ain2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, bin1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, bin2_Pin, GPIO_PIN_RESET);

    HAL_TIM_PWM_Start(pwm_tim, pwm_left_channel); //开启通道2的pwm输出
    HAL_TIM_PWM_Start(pwm_tim, pwm_right_channel); //开启通道2的pwm输出

    HAL_TIM_Base_Start_IT(encoder_count_tim);      //开启10ms中断

    HAL_TIM_Encoder_Start(encoder_left_tim, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(encoder_right_tim, TIM_CHANNEL_ALL);
    __HAL_TIM_ENABLE_IT(encoder_left_tim, TIM_IT_UPDATE);   //开启编码器定时器更新中断,防溢出处理
    __HAL_TIM_ENABLE_IT(encoder_right_tim, TIM_IT_UPDATE);  //开启编码器定时器更新中断,防溢出处理

    while (mpu_dmp_init());  //MPU DMP初始化
}

/*
 * 主循环,任务分发
 */
void UserLoop() {
    if (flag) {
        if (recv_buffer[0] != 0xff) {
            return;
        }
        uint8_t buff[4] = {recv_buffer[5], recv_buffer[4], recv_buffer[3],
                           recv_buffer[2]};
        switch (recv_buffer[1]) {
            case 0x00:
                upright_pid.kp = *(float *) buff;
                //pf("set kp= %f\n", upright_pid.kp);
                break;
            case 0x01:
                upright_pid.kd = *(float *) buff;
                //pf("set kd= %f\n", upright_pid.kd);
                break;
            case 0x02:
                upright_pid.target = *(float *) buff;
                //pf("set angle= %f\n", upright_pid.target);
                break;
            case 0x03:
                speed_pid.kp = *(float *) buff;
                //pf("set kp= %f\n", speed_pid.kp);
                break;
            case 0x04:
                speed_pid.ki = *(float *) buff;
                //pf("set ki= %f\n", speed_pid.ki);
                break;
            case 0x05:
                speed_pid.target = *(float *) buff;
                //pf("set speed= %f\n", speed_pid.target);
                break;
            default:
                break;
        }
        flag = 0;
    }
}

/*
 * mpu6050中断回调函数
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == GPIO_PIN_1) {
        if (mpu_dmp_get_data(&pitch, &roll, &yaw) == 0) {
//            MPU_Get_Accelerometer(&aacx, &aacy, &aacz);	//得到加速度传感器数据
            MPU_Get_Gyroscope(&gyrox, &gyroy, &gyroz);	//得到陀螺仪数据
//            float Velocity_out = Velocity(&speed_pid, speed_left + speed_right);
//            int pwm = (int) balance(&upright_pid, pitch + Velocity_out,
//                                    0);
            float Velocity_out = Velocity(&speed_pid, speed_left + speed_right);
            float balance_out = balance(&upright_pid, pitch,
                                        gyroy);
            int pwm_ctr = (int)(Velocity_out + balance_out);
            MotorControl(pwm_ctr, pwm_ctr);
            pf("%f,%d,%f\n", pitch, Velocity_out,balance_out);
        }
    }
}


/*
 * 定时器中断
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim == (encoder_count_tim)) {
        //  获取编码器信号数
        short encoder_left = __HAL_TIM_GET_COUNTER(encoder_left_tim);
        short encoder_right = __HAL_TIM_GET_COUNTER(encoder_right_tim);
        //  编码器数据清零
        __HAL_TIM_SET_COUNTER(encoder_left_tim, 0);
        __HAL_TIM_SET_COUNTER(encoder_right_tim, 0);
        //  计算转速
        speed_left = ((float) encoder_left);
        speed_right = ((float) encoder_right);
    }
}

/*
 * 串口空闲中断
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *handleTypeDef, uint16_t Size) {

    if (handleTypeDef->Instance == USART2) {
        //HAL_UART_Transmit(debug_uart, recv_buffer, Size, 0xff);
        flag = 1;
        HAL_UARTEx_ReceiveToIdle_IT(debug_uart, recv_buffer,
                                    sizeof(recv_buffer));
    }
}
