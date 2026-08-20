#ifndef SERVO_H
#define SERVO_H

#include "main.h"
#include <stdint.h>

/* 도어 개폐 각도 - 실제 도어 링키지 구조에 맞게 조정 */
#define SERVO_CLOSE_ANGLE   0
#define SERVO_OPEN_ANGLE    90

void Servo_Init(void);
void Servo_SetAngle(uint16_t angle);
void Servo_Open(void);   // 정방향 - 문 열기
void Servo_Close(void);  // 역방향 - 문 닫기

#endif /* SERVO_H */