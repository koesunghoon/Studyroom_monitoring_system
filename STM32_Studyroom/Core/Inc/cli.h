/*
 * cli.h
 *
 * USART2(ST-LINK 가상 COM포트)를 통한 간단한 CLI
 * PC에서 시리얼 터미널(PlatformIO Monitor, PuTTY 등)로 접속해서
 * 메뉴 선택 -> 지문 등록/인식 테스트 가능
 */

#ifndef CLI_H
#define CLI_H

#include "main.h"

void CLI_Init(void);
void CLI_Run(void);   // 무한루프 안에서 반복 호출 (메뉴 출력 -> 입력대기 -> 실행)

#endif /* CLI_H */