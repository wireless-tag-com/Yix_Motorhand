
#ifndef OPEN_LOOP_H
#define OPEN_LOOP_H

#include <stdint.h>
#include <math.h>
#include "driver/ledc.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "driver/gpio.h"


#define PI                      3.14159265358979323846
#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO_0        (17)    // 电机A相PWM引脚（根据硬件修改）
#define LEDC_OUTPUT_IO_1        (10)    // 电机B相PWM引脚（根据硬件修改）
#define LEDC_OUTPUT_IO_2        (8)    // 电机C相PWM引脚（根据硬件修改）
#define LEDC_CHANNEL_0          LEDC_CHANNEL_0
#define LEDC_CHANNEL_1          LEDC_CHANNEL_1
#define LEDC_CHANNEL_2          LEDC_CHANNEL_2
#define LEDC_DUTY_RES           LEDC_TIMER_8_BIT  // 8位占空比（0-255）
#define LEDC_FREQUENCY          (20000) // PWM频率20kHz
#define GPIO_DRIVER_ENABLE_PIN  GPIO_NUM_9 // 电机驱动器使能引脚（高电平使能）
#define MOTOR_POLE_PAIRS        7       // 电机极对数（根据实际电机修改）
#define VOLTAGE_POWER_SUPPLY    12.6f   // 电源电压（V）


/**
 * @brief 初始化 LEDC PWM 外设（用于电机三相驱动）
 */
void OpenLoop_LEDCPWM_Init(void);

/**
 * @brief 初始化电机驱动器使能引脚（GPIO输出高电平）
 */
void OpenLoop_DriverEnable_Init(void);

/**
 * @brief 电机开环速度控制（核心函数）
 * @param target_velocity 目标机械角速度（rad/s）
 * @return 当前输出的Q轴电压（Uq）
 */
float OpenLoop_VelocityControl(float target_velocity);  // 补全函数声明

#endif // OPEN_LOOP_H