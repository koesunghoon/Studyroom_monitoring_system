#include "esp01.h"
#include "usart.h"
#include <string.h>
#include <stdio.h>

extern UART_HandleTypeDef huart6;
#define ESP01_UART   (&huart6)

#define ESP01_RESP_BUF_SIZE   256
#define ESP01_IPD_BUF_SIZE    128
#define ESP01_RXRING_SIZE     256

static char respBuf[ESP01_RESP_BUF_SIZE];
static char ipdBuf[ESP01_IPD_BUF_SIZE];
static uint16_t ipdIdx = 0;

/* ---- 인터럽트 수신용 링버퍼 ---- */
static volatile uint8_t rxRing[ESP01_RXRING_SIZE];
static volatile uint16_t rxHead = 0, rxTail = 0;
static uint8_t rxByte; // IT 수신용 1바이트 홀더

static void ESP01_RxStart(void)
{
    HAL_UART_Receive_IT(&huart6, &rxByte, 1);
}

// stm32f4xx_it.c의 USART6_IRQHandler -> HAL_UART_IRQHandler가 호출해줌
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART6) {
        uint16_t next = (rxHead + 1) % ESP01_RXRING_SIZE;
        if (next != rxTail) { // 링버퍼 안 꽉 찼으면
            rxRing[rxHead] = rxByte;
            rxHead = next;
        }
        HAL_UART_Receive_IT(&huart6, &rxByte, 1); // 다음 바이트 위해 즉시 재무장
    }
}

static uint8_t RingBuf_Get(uint8_t *out)
{
    if (rxHead == rxTail) return 0; // 비어있음
    *out = rxRing[rxTail];
    rxTail = (rxTail + 1) % ESP01_RXRING_SIZE;
    return 1;
}

static void ESP01_Print(const char *str)
{
    HAL_UART_Transmit(ESP01_UART, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);
}

/* [수정] 폴링 HAL_UART_Receive 대신 링버퍼에서 꺼내오도록 변경 */
static uint8_t ESP01_WaitResponse(const char *expect, uint32_t timeout_ms)
{
    uint16_t idx = 0;
    uint8_t ch;
    uint32_t startTick = HAL_GetTick();

    memset(respBuf, 0, sizeof(respBuf));

    while ((HAL_GetTick() - startTick) < timeout_ms) {
        if (RingBuf_Get(&ch)) {
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

uint8_t ESP01_SendCommand(const char *cmd, const char *expect, uint32_t timeout_ms)
{
    ESP01_Print(cmd);
    ESP01_Print("\r\n");
    return ESP01_WaitResponse(expect, timeout_ms);
}

void ESP01_Init(void)
{
    ESP01_RxStart(); // [추가] 인터럽트 수신 시작 (이후로는 절대 안 멈춤)

    ESP01_SendCommand("AT", "OK", 2000);
    ESP01_SendCommand("ATE0", "OK", 2000);
    ESP01_SendCommand("AT+CWMODE=1", "OK", 2000);
    ESP01_SendCommand("AT+CIPMUX=0", "OK", 2000);
}

uint8_t ESP01_ConnectWiFi(const char *ssid, const char *password)
{
    char cmd[128];
    snprintf(cmd, sizeof(cmd), "AT+CWJAP=\"%s\",\"%s\"", ssid, password);
    return ESP01_SendCommand(cmd, "WIFI GOT IP", 15000);
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
    result = ESP01_SendCommand(cmd, ">", 5000);
    if (result != ESP01_OK) return result;

    ESP01_Print(data);
    return ESP01_WaitResponse("SEND OK", 5000);
}

uint8_t ESP01_CloseConnection(void)
{
    return ESP01_SendCommand("AT+CIPCLOSE", "OK", 3000);
}

/* [수정] 폴링 HAL_UART_Receive 대신 링버퍼에서 꺼내오도록 변경
   -> 이제 Servo_Open()의 3초 딜레이 중에도 인터럽트가 계속 바이트를 받아 쌓아두므로
      절대 유실되지 않음 */
uint8_t ESP01_CheckIncomingData(char *outBuf, uint16_t outBufSize)
{
    uint8_t ch;

    while (RingBuf_Get(&ch)) {
        if (ipdIdx < ESP01_IPD_BUF_SIZE - 1) {
            ipdBuf[ipdIdx++] = ch;
            ipdBuf[ipdIdx] = '\0';
        } else {
            ipdIdx = 0;
            continue;
        }

        char *header = strstr(ipdBuf, "+IPD,");
        if (header == NULL) continue;

        char *colon = strchr(header, ':');
        if (colon == NULL) continue;

        int dataLen = atoi(header + 5);
        int received = ipdIdx - (int)(colon - ipdBuf) - 1;

        if (received < dataLen) continue;

        strncpy(outBuf, colon + 1, outBufSize - 1);
        outBuf[outBufSize - 1] = '\0';

        ipdIdx = 0;
        return 1;
    }

    return 0;
}

const char* ESP01_GetLastResponse(void)
{
    return respBuf;
}