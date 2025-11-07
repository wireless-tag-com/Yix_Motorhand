/*********************************************************/
/*********************************************************/
/*************************开环*************************/
/*********************************************************/
/*********************************************************/
/*#include "OpenLoop.h"  
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// ESP-IDF 入口函数（C链接，确保编译器识别）
extern "C" void app_main(void) {
    // 1. 初始化电机驱动器使能引脚
    OpenLoop_DriverEnable_Init();

    // 2. 初始化 LEDC PWM 外设（用于电机三相驱动）
    OpenLoop_LEDCPWM_Init();

    // 3. 开环速度控制主循环（持续运行）
    while (1) {
        // 调用组件接口，目标速度40 rad/s（可根据需求修改）
        OpenLoop_VelocityControl(40.0f);

        // 延时10ms（控制周期10ms，避免CPU占用过高）
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}*/
  //已验证

/*********************************************************/
/*********************************************************/
/*************************力矩闭环*************************/
/*********************************************************/
/*********************************************************/

/*#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "FOC.h"
#include "AS5600.h"
#include "PowerLoop.h"
#include <math.h>

// 定义电机参数
#define POLE_PAIRS 7
#define DIRECTION 1
#define POWER_SUPPLY 12.6f

// 定义PWM引脚和通道
#define PWM_A_GPIO GPIO_NUM_17
#define PWM_B_GPIO GPIO_NUM_10
#define PWM_C_GPIO GPIO_NUM_8
#define PWM_A_CHANNEL LEDC_CHANNEL_0
#define PWM_B_CHANNEL LEDC_CHANNEL_1
#define PWM_C_CHANNEL LEDC_CHANNEL_2

// 定义使能引脚
#define ENABLE_GPIO GPIO_NUM_9

// 全局变量
FOC foc(PWM_A_GPIO, PWM_B_GPIO, PWM_C_GPIO,
        PWM_A_CHANNEL, PWM_B_CHANNEL, PWM_C_CHANNEL,
        POLE_PAIRS, DIRECTION);

// 创建力矩环实例（传递指针）
PowerLoop power_loop(&foc, 0.5f, 0.01f, 0.05f, 5.0f, 3.0f);

// 目标力矩设置
#define TARGET_TORQUE 1.0f

// 状态打印间隔（毫秒）
#define PRINT_INTERVAL_MS 1000
uint32_t last_print_time = 0;

// 设置使能引脚高电平
void set_enable_high(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << ENABLE_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    gpio_set_level(ENABLE_GPIO, 1);
    printf("驱动器使能已打开\n");
}

extern "C" void app_main(void)
{
    // 设置使能引脚高电平
    set_enable_high();
    
    // 初始化FOC
    printf("初始化FOC...\n");
    if (foc.begin(POWER_SUPPLY) != ESP_OK) {
        printf("FOC初始化失败!\n");
        return;
    }
    printf("FOC初始化成功!\n");

    // 初始化力矩环
    printf("初始化力矩环...\n");
    if (power_loop.begin() != ESP_OK) {
        printf("力矩环初始化失败!\n");
        return;
    }

    // 设置目标力矩
    power_loop.setTargetTorque(TARGET_TORQUE);
    printf("目标力矩: %.2f N·m\n", TARGET_TORQUE);

    // 传感器对齐
    printf("开始传感器对齐...\n");
    foc.alignSensor();
    vTaskDelay(pdMS_TO_TICKS(1000));
    printf("传感器对齐完成!\n");

    printf("开始力矩控制...\n");

    while (1) {
        // 运行力矩环控制（使用torqueLoop而不是run）
        power_loop.torqueLoop();

        // 定期打印状态信息
        uint32_t current_time = esp_timer_get_time() / 1000;
        if (current_time - last_print_time > PRINT_INTERVAL_MS) {
            printf("目标力矩: %.2f N·m, 当前力矩: %.2f N·m\n",
                   TARGET_TORQUE, power_loop.getCurrentTorque());
            last_print_time = current_time;
        }

        vTaskDelay(pdMS_TO_TICKS(10));  // 控制周期10ms (100Hz)
    }
}*/



/*********************************************************/
/*********************************************************/
/*************************速度闭环*************************/
/*********************************************************/
/*********************************************************/
/*#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "FOC.h"
#include "SpeedLoop.h"
#include <math.h>

// 定义电机参数
#define POLE_PAIRS 7
#define DIRECTION 1
#define POWER_SUPPLY 12.6f

// 定义PWM引脚和通道
#define PWM_A_GPIO GPIO_NUM_17
#define PWM_B_GPIO GPIO_NUM_10
#define PWM_C_GPIO GPIO_NUM_8
#define PWM_A_CHANNEL LEDC_CHANNEL_0
#define PWM_B_CHANNEL LEDC_CHANNEL_1
#define PWM_C_CHANNEL LEDC_CHANNEL_2

// 定义使能引脚
#define ENABLE_GPIO GPIO_NUM_9

// 全局对象
FOC foc(PWM_A_GPIO, PWM_B_GPIO, PWM_C_GPIO,
        PWM_A_CHANNEL, PWM_B_CHANNEL, PWM_C_CHANNEL,
        POLE_PAIRS, DIRECTION);

SpeedLoop speed_loop(&foc);  // 使用默认PID参数

// 目标速度设置（弧度/秒）
#define TARGET_VELOCITY 2.0f

// 状态打印间隔（毫秒）
#define PRINT_INTERVAL_MS 1000
uint32_t last_print_time = 0;

// 设置使能引脚高电平
void set_enable_high(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << ENABLE_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    gpio_set_level(ENABLE_GPIO, 1);
    printf("驱动器使能已打开\n");
}

extern "C" void app_main(void)
{
    // 设置使能引脚高电平
    set_enable_high();
    
    // 初始化FOC
    printf("初始化FOC...\n");
    if (foc.begin(POWER_SUPPLY) != ESP_OK) {
        printf("FOC初始化失败!\n");
        return;
    }
    printf("FOC初始化成功!\n");

    // 传感器对齐
    printf("开始传感器对齐...\n");
    foc.alignSensor();
    vTaskDelay(pdMS_TO_TICKS(1000));
    printf("传感器对齐完成!\n");

    // 初始化速度环
    printf("初始化速度环...\n");
    if (speed_loop.begin() != ESP_OK) {
        printf("速度环初始化失败!\n");
        return;
    }

    // 设置目标速度
    speed_loop.setTargetVelocity(TARGET_VELOCITY);
    speed_loop.enable(true);

    printf("开始闭环速度控制...\n");
    printf("目标速度: %.2f rad/s\n", TARGET_VELOCITY);

    while (1) {
        // 更新速度环控制
        speed_loop.update();

        // 定期打印状态信息
        uint32_t current_time = esp_timer_get_time() / 1000;
        if (current_time - last_print_time > PRINT_INTERVAL_MS) {
            printf("目标: %.2f rad/s, 当前: %.2f rad/s, 误差: %.3f rad/s, Uq: %.2f V\n",
                   speed_loop.getTargetVelocity(),
                   speed_loop.getCurrentVelocity(),
                   speed_loop.getVelocityError(),
                   speed_loop.getControlOutput());
            last_print_time = current_time;
        }

        vTaskDelay(pdMS_TO_TICKS(10));  // 控制周期10ms (100Hz)
    }
}//已测试
*/

/*********************************************************/
/*********************************************************/
/*************************电流闭环*************************/
/*********************************************************/
/*********************************************************/
/*include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "FOC.h"
#include "CurrentLoop.h"
#include "SpeedLoop.h"
#include <math.h>

// 定义电机参数
#define POLE_PAIRS 7
#define DIRECTION -1
#define POWER_SUPPLY 12.6f

// 定义PWM引脚和通道
#define PWM_A_GPIO GPIO_NUM_17
#define PWM_B_GPIO GPIO_NUM_10
#define PWM_C_GPIO GPIO_NUM_8
#define PWM_A_CHANNEL LEDC_CHANNEL_0
#define PWM_B_CHANNEL LEDC_CHANNEL_1
#define PWM_C_CHANNEL LEDC_CHANNEL_2

// 定义使能引脚
#define ENABLE_GPIO GPIO_NUM_9

// 更保守的PID参数
#define CURRENT_Q_KP 0.5f    // 降低比例系数
#define CURRENT_Q_KI 0.1f    // 降低积分系数  
#define CURRENT_Q_KD 0.0f
#define CURRENT_Q_RAMP 100.0f  // 降低电压变化率

#define CURRENT_D_KP 0.5f
#define CURRENT_D_KI 0.1f
#define CURRENT_D_KD 0.0f
#define CURRENT_D_RAMP 100.0f

// 全局对象
FOC foc(PWM_A_GPIO, PWM_B_GPIO, PWM_C_GPIO,
        PWM_A_CHANNEL, PWM_B_CHANNEL, PWM_C_CHANNEL,
        POLE_PAIRS, DIRECTION);

CurrentLoop current_loop(&foc, 
                       CURRENT_Q_KP, CURRENT_Q_KI, CURRENT_Q_KD,
                       CURRENT_D_KP, CURRENT_D_KI, CURRENT_D_KD);

SpeedLoop speed_loop(&foc);      // 速度环控制器（可选）

// 状态打印间隔（毫秒）
#define PRINT_INTERVAL_MS 1000
uint32_t last_print_time = 0;

// 电流环控制周期（毫秒）
#define CURRENT_LOOP_PERIOD_MS 1  // 1ms周期，1kHz频率

// 紧急停止标志
bool emergency_stop = false;

// 设置使能引脚高电平
void set_enable_high(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << ENABLE_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    gpio_set_level(ENABLE_GPIO, 1);
    printf("驱动器使能已打开\n");
}

// 紧急停止函数
void emergency_stop_motor() {
    emergency_stop = true;
    current_loop.enable(false);
    foc.setTorque(0, 0);  // 停止输出
    printf("紧急停止！电机已禁用\n");
}

// 电流环控制任务
void current_control_task(void* arg) {
    TickType_t last_wake_time = xTaskGetTickCount();
    
    while (1) {
        if (!emergency_stop) {
            current_loop.update();
        } else {
            // 紧急停止状态下，确保输出为零
            foc.setTorque(0, 0);
        }
        
        // 精确的1ms周期控制
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(CURRENT_LOOP_PERIOD_MS));
    }
}

// 安全约束函数
float safe_constrain(float value, float min, float max) {
    return (value < min) ? min : (value > max) ? max : value;
}

extern "C" void app_main(void)
{
    // 设置使能引脚高电平
    set_enable_high();
    
    // 初始化FOC
    printf("初始化FOC...\n");
    if (foc.begin(POWER_SUPPLY) != ESP_OK) {
        printf("FOC初始化失败!\n");
        return;
    }
    printf("FOC初始化成功!\n");

    // 先不启用电流采样，避免干扰
    foc.enableCurrentSampling(false);

    // 传感器对齐
    printf("开始传感器对齐...\n");
    foc.alignSensor();
    vTaskDelay(pdMS_TO_TICKS(1000));
    printf("传感器对齐完成!\n");

    // 初始化电流环
    printf("初始化电流环...\n");
    if (current_loop.begin() != ESP_OK) {
        printf("电流环初始化失败!\n");
        return;
    }
    printf("电流环初始化成功!\n");

    // 启用电流采样
    foc.enableCurrentSampling(true);

    // 先设置小电流测试
    float test_current = 0.1f;  // 从0.1A开始测试
    current_loop.setTargetCurrent(test_current, 0.0f);
    current_loop.enable(true);

    printf("开始小电流测试: %.2fA\n", test_current);
    printf("电流环控制频率: %dHz\n", 1000 / CURRENT_LOOP_PERIOD_MS);
    printf("安全保护已启用，最大电流限制为2A\n");

    // 创建电流环控制任务（高优先级）
    xTaskCreate(current_control_task, "current_ctrl", 4096, NULL, 6, NULL);

    // 可选：初始化速度环（用于监控）
    speed_loop.begin();
    speed_loop.enable(false); // 不启用速度控制，只用于监控

    // 测试阶段控制
    int test_phase = 0;
    uint32_t phase_start_time = 0;
    const uint32_t phase_duration = 5000; // 每个测试阶段5秒

    while (1) {
        // 处理串口命令
        foc.serialReceiveUserCommand();
        
        // 检查是否有新的目标值通过串口输入
        float motor_target = foc.getMotorTarget();
        if (motor_target != 0) {
            // 安全限制：最大电流2A
            float safe_target = safe_constrain(motor_target, -2.0f, 2.0f);
            if (safe_target != motor_target) {
                printf("警告：目标电流过大，已限制为%.2fA\n", safe_target);
            }
            current_loop.setTargetCurrentIq(safe_target);
            printf("更新目标电流: %.3fA\n", safe_target);
        }

        // 自动测试阶段（可选）
        uint32_t current_time = esp_timer_get_time() / 1000;
        if (current_time - phase_start_time > phase_duration && test_phase < 3) {
            test_phase++;
            phase_start_time = current_time;
            
            switch(test_phase) {
                case 1:
                    test_current = 0.2f;
                    break;
                case 2:
                    test_current = 0.5f;
                    break;
                case 3:
                    test_current = 1.0f;
                    break;
            }
            current_loop.setTargetCurrentIq(test_current);
            printf("进入测试阶段 %d，目标电流: %.2fA\n", test_phase, test_current);
        }

        // 定期打印状态信息
        if (current_time - last_print_time > PRINT_INTERVAL_MS) {
            float current_velocity = foc.getVelocity();
            float electrical_angle = foc.getElectricalAngle();
            
            printf("电流状态 - Iq: %.3f/%.3fA, Id: %.3f/%.3fA, 误差: %.3fA\n",
                   current_loop.getCurrentIq(), current_loop.getTargetIq(),
                   current_loop.getCurrentId(), current_loop.getTargetId(),
                   current_loop.getCurrentErrorIq());
            
            printf("系统状态 - 速度: %.3frad/s, 电角度: %.2frad, Uq: %.2fV, Ud: %.2fV\n",
                   current_velocity, electrical_angle,
                   current_loop.getControlOutputIq(),
                   current_loop.getControlOutputId());
            
            // 安全监控
            float output_voltage = fabs(current_loop.getControlOutputIq());
            if (output_voltage > 3.0f) {
                printf("警告: 控制电压过高(%.2fV)，可能引起电机发热!\n", output_voltage);
            }
            
            if (fabs(current_loop.getCurrentIq()) > 2.5f) {
                printf("警告: 检测到过大电流(%.2fA)，考虑紧急停止!\n", current_loop.getCurrentIq());
            }
            
            printf("测试阶段: %d/3, 运行时间: %lus\n", test_phase, current_time/1000);
            printf("----------------------------------------\n");
            last_print_time = current_time;
        }

        // 手动紧急停止检查（可以通过串口命令实现）
        // 这里可以添加其他紧急停止条件
        
        vTaskDelay(pdMS_TO_TICKS(100));  // 主循环100ms周期
    }
}*/