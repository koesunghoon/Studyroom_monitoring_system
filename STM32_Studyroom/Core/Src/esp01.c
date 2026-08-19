/*
 * esp01.c
 *
 * ESP-01 WiFi 모듈 드라이버 구현 (AT 커맨드, USART6 + HAL)
 */

#include "esp01.h"
#include "usart.h"
#include <string.h>
#include <stdio.h>

extern UART_HandleTypeDef huart6;
#define ESP01_UART   (&huart6)

#define ESP01_RESP_BUF_SIZE   256
static char respBuf[ESP01_RESP_BUF_SIZE];

static void ESP01_Print(const char *str)
{
    HAL_UART_Transmit(ESP01_UART, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);
}

/*
 * ESP01_WaitResponse
 *  아무것도 보내지 않고, 응답에 expect 문자열이 나올 때까지 대기
 *  (CIPSEND 이후 실제 데이터 보내고 "SEND OK" 기다릴 때처럼 명령 전송 없이 대기만 필요한 경우용)
 */
static uint8_t ESP01_WaitResponse(const char *expect, uint32_t timeout_ms)
{
    uint16_t idx = 0;
    uint8_t ch;
    uint32_t startTick = HAL_GetTick();

    memset(respBuf, 0, sizeof(respBuf));

    while ((HAL_GetTick() - startTick) < timeout_ms) {
        if (HAL_UART_Receive(ESP01_UART, &ch, 1, 50) == HAL_OK) {
            if (idx < ESP01_RESP_BUF_SIZE - 1) {
                respBuf[idx++] = ch;
                respBuf[idx] = '\0';
            }

            if (expect != NULL && strstr(respBuf, expect) != NULL) {
                return ESP01_OK;
            }
            if (strstr(respBuf, "ERROR") != NULL) {
                return ESP01_ERROR;
            }
        }
    }

    return ESP01_TIMEOUT;
}

/*
 * ESP01_SendCommand
 *  AT 커맨드 전송 후 expect 문자열 나올 때까지 대기
 */
uint8_t ESP01_SendCommand(const char *cmd, const char *expect, uint32_t timeout_ms)
{
    ESP01_Print(cmd);
    ESP01_Print("\r\n");
    return ESP01_WaitResponse(expect, timeout_ms);
}

void ESP01_Init(void)
{
    ESP01_SendCommand("AT", "OK", 2000);          // 모듈 응답 확인
    ESP01_SendCommand("ATE0", "OK", 2000);         // 에코 끄기
    ESP01_SendCommand("AT+CWMODE=1", "OK", 2000);  // Station 모드
    ESP01_SendCommand("AT+CIPMUX=0", "OK", 2000);  // 단일 연결 모드
}

uint8_t ESP01_ConnectWiFi(const char *ssid, const char *password)
{
    char cmd[128];
    snprintf(cmd, sizeof(cmd), "AT+CWJAP=\"%s\",\"%s\"", ssid, password);
    return ESP01_SendCommand(cmd, "WIFI GOT IP", 15000); // AP 접속은 시간 걸릴 수 있어 15초
}

uint8_t ESP01_ConnectServer(const char *ip, uint16_t port)
{
    char cmd[64];
    snprintf(cmd, sizeof(cmd), "AT+CIPSTART=\"TCP\",\"%s\",%d", ip, port);
    return ESP01_SendCommand(cmd, "CONNECT", 10000);
}

uint8_t ESP01_SendData(const char *data, uint16_t len)
{
    char cmd[32];
    uint8_t result;

    snprintf(cmd, sizeof(cmd), "AT+CIPSEND=%d", len);
    result = ESP01_SendCommand(cmd, ">", 5000); // ESP가 데이터 받을 준비되면 '>' 프롬프트 응답
    if (result != ESP01_OK) return result;

    ESP01_Print(data); // 실제 페이로드는 순수 데이터만 전송 (\r\n 붙이지 않음)
    return ESP01_WaitResponse("SEND OK", 5000);
}

uint8_t ESP01_CloseConnection(void)
{
    return ESP01_SendCommand("AT+CIPCLOSE", "OK", 3000);
}

const char* ESP01_GetLastResponse(void)
{
    return respBuf;
}

