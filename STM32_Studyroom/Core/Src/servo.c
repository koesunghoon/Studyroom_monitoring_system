#include "servo.h"
#include "tim.h"

extern TIM_HandleTypeDef htim3;
#define SERVO_TIM      (&htim3)
#define SERVO_CHANNEL  TIM_CHANNEL_1

#define SERVO_PULSE_MIN   500  // 0도   (1.0ms)
#define SERVO_PULSE_MAX   3000  // 180도 (2.0ms)

void Servo_Init(void)
{
    HAL_TIM_PWM_Start(SERVO_TIM, SERVO_CHANNEL);
    Servo_SetAngle(SERVO_CLOSE_ANGLE); // 부팅 시 초기 상태 = 닫힘
}

void Servo_SetAngle(uint16_t angle)
{
    if (angle > 180) angle = 180;

    uint32_t pulse = SERVO_PULSE_MIN +
        ((uint32_t)(SERVO_PULSE_MAX - SERVO_PULSE_MIN) * angle) / 180;

    __HAL_TIM_SET_COMPARE(SERVO_TIM, SERVO_CHANNEL, pulse);
}

void Servo_Open(void)
{
    Servo_SetAngle(SERVO_OPEN_ANGLE);
}

void Servo_Close(void)
{
    Servo_SetAngle(SERVO_CLOSE_ANGLE);
}