/*
 * esp01.h
 *
 * ESP-01 WiFi 모듈 드라이버 (AT 커맨드, USART6 기반)
 * TCP 클라이언트로 자체 웹서버에 접속
 */

#ifndef ESP01_H
#define ESP01_H

#include "main.h"
#include <stdint.h>

/* ---------------- WiFi / 서버 접속 정보 (실제 값으로 교체) ---------------- */
#define WIFI_SSID       "turtlebot"
#define WIFI_PASSWORD   "turtlebot"
#define SERVER_IP       "192.168.0.2"
#define SERVER_PORT     5000

/* ---------------- 결과 코드 ---------------- */
#define ESP01_OK        0
#define ESP01_ERROR     1
#define ESP01_TIMEOUT   2

void    ESP01_Init(void);
uint8_t ESP01_SendCommand(const char *cmd, const char *expect, uint32_t timeout_ms);
uint8_t ESP01_ConnectWiFi(const char *ssid, const char *password);
uint8_t ESP01_ConnectServer(const char *ip, uint16_t port);
uint8_t ESP01_SendData(const char *data, uint16_t len);
uint8_t ESP01_CloseConnection(void);
const char* ESP01_GetLastResponse(void);

#endif /* ESP01_H */