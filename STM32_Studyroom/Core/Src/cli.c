/*
 * cli.c
 *
 * 시리얼 터미널 CLI 구현
 * USART2 (ST-LINK VCP) 사용, Baudrate는 usart.c의 CubeMX 설정값 그대로 사용 (보통 115200)
 */

#include "cli.h"
#include "fingerprint.h"
#include "rfid.h"
#include "usart.h"
#include "esp01.h"
#include "servo.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define CLI_UART   (&huart2)

/* ------------- 저수준 입출력 헬퍼 ------------- */

// 문자열 출력 (printf 대신 직접 UART로. printf 리다이렉트 설정 안 해도 동작하게)
static void CLI_Print(const char *str)
{
    HAL_UART_Transmit(CLI_UART, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);
}

// 한 줄 입력 받기 (Enter 칠 때까지 블로킹, 백스페이스 처리 포함)
// buf: 결과 저장할 버퍼, maxLen: buf 크기
static void CLI_ReadLine(char *buf, uint16_t maxLen)
{
    uint16_t idx = 0;
    uint8_t ch;

    memset(buf, 0, maxLen);

    while (1) {
        if (HAL_UART_Receive(CLI_UART, &ch, 1, HAL_MAX_DELAY) != HAL_OK) {
            continue;
        }

        if (ch == '\r' || ch == '\n') {
            CLI_Print("\r\n");

            // 터미널이 Enter를 \r\n 두 바이트로 보내는 경우가 많음.
            // 방금 \r 을 받았다면 뒤따라오는 \n 을(혹은 그 반대) 짧은 타임아웃으로 마저 비워준다.
            // 안 그러면 다음 CLI_ReadLine() 호출이 이 찌꺼기 바이트를 "빈 입력"으로 잘못 읽는다.
            uint8_t nextCh;
            uint8_t otherCh = (ch == '\r') ? '\n' : '\r';
            if (HAL_UART_Receive(CLI_UART, &nextCh, 1, 20) == HAL_OK) {
                if (nextCh != otherCh) {
                    // 개행 짝이 아닌 다른 문자였다면, 다음 루프를 위해 그냥 버림
                    // (간단한 구현이라 별도 버퍼링은 하지 않음)
                }
            }
            break;
        } else if (ch == 0x08 || ch == 0x7F) { // 백스페이스 처리
            if (idx > 0) {
                idx--;
                CLI_Print("\b \b"); // 화면에서 한 글자 지우기
            }
        } else if (idx < maxLen - 1) {
            buf[idx++] = ch;
            HAL_UART_Transmit(CLI_UART, &ch, 1, HAL_MAX_DELAY); // 에코
        }
    }

    buf[idx] = '\0';
}

// AS608 확인코드(Confirmation Code)를 사람이 읽을 수 있는 한글 메시지로 변환
static const char* CLI_ErrorToString(uint8_t code)
{
    switch (code) {
        case AS608_OK:                return "성공";
        case AS608_ERR_PACKET:        return "패킷 통신 오류";
        case AS608_ERR_NO_FINGER:     return "손가락이 감지되지 않음";
        case AS608_ERR_ENROLL_FAIL:   return "이미지 등록 실패";
        case AS608_ERR_DISORDER:      return "지문 이미지가 너무 지저분함";
        case AS608_ERR_SMALL_FEATURE: return "특징점 부족 (이미지 품질 낮음)";
        case AS608_ERR_NOMATCH:       return "지문 불일치";
        case AS608_ERR_NOTFOUND:      return "일치하는 지문 없음 (미등록 지문)";
        case AS608_ERR_COMBINE_FAIL:  return "두 스캔이 서로 다른 손가락으로 판단됨 (결합 실패)";
        case AS608_ERR_BAD_LOCATION:  return "잘못된 저장 슬롯 번호";
        case AS608_ERR_TIMEOUT:       return "통신 타임아웃 (배선/전원 확인 필요)";
        default:                      return "알 수 없는 오류";
    }
}

/* ------------- 메뉴 동작 ------------- */

static void CLI_MenuEnroll(void)
{
    char inputBuf[16];
    uint16_t pageID;
    uint8_t result;
    char msg[80];

    CLI_Print("\r\n===== 지문 등록 =====\r\n");
    CLI_Print("저장할 슬롯 번호 입력 (0~299): ");
    CLI_ReadLine(inputBuf, sizeof(inputBuf));
    pageID = (uint16_t)atoi(inputBuf);

    CLI_Print("\r\n[1/2] 손가락을 센서에 올려주세요...\r\n");
    result = AS608_WaitFinger(10000);
    snprintf(msg, sizeof(msg), "  WaitFinger: %s (0x%02X)\r\n", CLI_ErrorToString(result), result);
    CLI_Print(msg);
    if (result != AS608_OK) return;

    result = AS608_GenChar(1);
    snprintf(msg, sizeof(msg), "  GenChar(1): %s (0x%02X)\r\n", CLI_ErrorToString(result), result);
    CLI_Print(msg);
    if (result != AS608_OK) return;

    CLI_Print("\r\n손가락을 떼주세요...\r\n");
    result = AS608_WaitFingerRemoved(10000);
    snprintf(msg, sizeof(msg), "  WaitFingerRemoved: %s\r\n", CLI_ErrorToString(result));
    CLI_Print(msg);
    if (result != AS608_OK) return;

    HAL_Delay(500); // 200 -> 500ms로 늘려서 테스트

    CLI_Print("\r\n[2/2] 같은 손가락을 다시 올려주세요...\r\n");
    result = AS608_WaitFinger(10000);
    snprintf(msg, sizeof(msg), "  WaitFinger(2): %s\r\n", CLI_ErrorToString(result));
    CLI_Print(msg);
    if (result != AS608_OK) return;

    result = AS608_GenChar(2);
    snprintf(msg, sizeof(msg), "  GenChar(2): %s\r\n", CLI_ErrorToString(result));
    CLI_Print(msg);
    if (result != AS608_OK) return;

    result = AS608_RegModel();
    snprintf(msg, sizeof(msg), "  RegModel: %s\r\n", CLI_ErrorToString(result));
    CLI_Print(msg);
    if (result != AS608_OK) return;

    result = AS608_StoreChar(1, pageID);
    snprintf(msg, sizeof(msg), "\r\n>> Store: %s (슬롯 %d)\r\n", CLI_ErrorToString(result), pageID);
    CLI_Print(msg);

    // CLI Print 추가
    CLI_Print("\r\n[알림] studycam 서버 students 테이블에도 이 슬롯 번호로\r\n");
    CLI_Print("       학생 정보를 등록해두어야 출결이 정상 기록됩니다.\r\n");
}

static void CLI_MenuVerify(void)
{
    uint8_t result;
    uint16_t matchID = 0, matchScore = 0;
    char msg[80];

    CLI_Print("\r\n===== 지문 인식 =====\r\n");
    CLI_Print("손가락을 센서에 올려주세요...\r\n");

    result = AS608_Verify(&matchID, &matchScore);

    if (result == AS608_OK) {
        snprintf(msg, sizeof(msg), "\r\n>> 인식 성공! 슬롯: %d, 매칭점수: %d\r\n",
                  matchID, matchScore);
        CLI_Print(msg);

        /* [추가] 서버(studycam)로 출결 결과 전송
           app.py가 5001번 포트에서 순수 TCP로 "FP,<슬롯번호>\n" 형식을 받아
           students 테이블 조회 -> attendance 테이블에 입실/퇴실 자동 기록함.
           연결이 안 되어 있으면(부팅 시 자동연결 실패 등) 실패 메시지만 출력하고
           지문 인식 자체는 정상 진행됨 (전송 실패가 인식 실패는 아님). */
        {
            char sendBuf[16];
            int sendLen = snprintf(sendBuf, sizeof(sendBuf), "FP,%d\n", matchID);
            uint8_t sendResult = ESP01_SendData(sendBuf, (uint16_t)sendLen);

            if (sendResult == ESP01_OK) {
                CLI_Print(">> 서버로 출결 전송 완료\r\n");
            } else {
                CLI_Print(">> 서버 전송 실패 (WiFi/서버 연결 상태 확인 필요, 메뉴 4번으로 재연결 가능)\r\n");
            }
        }
        
        CLI_Print(">> 문 열림\r\n");
        Servo_Open();
        HAL_Delay(3000); // 문 열려있는 시간 (필요에 맞게 조정)
        CLI_Print(">> 문 닫힘\r\n");
        Servo_Close();

    } else {
        snprintf(msg, sizeof(msg), "\r\n>> 인식 실패: %s (코드: 0x%02X)\r\n",
                  CLI_ErrorToString(result), result);
        CLI_Print(msg);
    }
}

static void CLI_MenuHandshake(void)
{
    // 주의: AS608_CMD_HANDSHAKE(0x35)는 일부 모듈에서 미지원일 수 있어
    // 모든 AS608 계열이 100% 지원하는 GetImage(0x01)로 통신 자체를 검증한다.
    // 손가락이 없는 상태에서 테스트하면 정상일 때 "손가락 없음(0x02)"이 응답으로 와야 함.
    // 이게 뜨면 = 모듈과의 UART 통신 자체는 정상이라는 뜻.
    uint8_t result = AS608_GetImage();
    char msg[100];

    if (result == AS608_OK) {
        CLI_Print("\r\n>> 통신 정상! (손가락이 이미 센서에 있어서 이미지 캡처됨)\r\n");
    } else if (result == AS608_ERR_NO_FINGER) {
        CLI_Print("\r\n>> 통신 정상! (손가락 없음 응답 수신 - UART 통신 확인됨)\r\n");
    } else if (result == AS608_ERR_TIMEOUT) {
        CLI_Print("\r\n>> 통신 실패: 응답 없음 (타임아웃). 배선/전원/baudrate 확인 필요\r\n");
    } else {
        snprintf(msg, sizeof(msg), "\r\n>> 예상 밖 응답 코드: 0x%02X (통신은 되지만 프로토콜 불일치 가능성)\r\n", result);
        CLI_Print(msg);
    }
}

static void CLI_MenuAdminAuth(void)
{
    uint8_t uid[RFID_UID_LEN];
    uint8_t status;
    char msg[100];

    CLI_Print("\r\n===== 관리자 인증 =====\r\n");
    CLI_Print("카드를 리더기에 태그해주세요...\r\n");

    uint32_t startTick = HAL_GetTick();
    do {
        status = RFID_ScanCard(uid);
        if (status == MI_OK) break;
        HAL_Delay(100);
    } while (HAL_GetTick() - startTick < 10000);

    if (status != MI_OK) {
        CLI_Print("\r\n>> 카드 인식 실패 (타임아웃 또는 통신 오류)\r\n");
        return;
    }

    snprintf(msg, sizeof(msg), "\r\n스캔된 UID: %02X %02X %02X %02X\r\n",
             uid[0], uid[1], uid[2], uid[3]);
    CLI_Print(msg);

    if (RFID_IsAdminCard(uid)) {
        CLI_Print(">> 인증 성공!\r\n");
    } else {
        CLI_Print(">> 인증 실패 (등록되지 않은 카드)\r\n");
    }
}

static void CLI_MenuWifiTest(void)
{
    uint8_t result;

    CLI_Print("\r\n===== WiFi 연결 테스트 =====\r\n");

    CLI_Print("ESP01 초기화 중...\r\n");
    ESP01_Init();

    CLI_Print("WiFi 접속 시도 중...\r\n");
    result = ESP01_ConnectWiFi(WIFI_SSID, WIFI_PASSWORD);
    if (result != ESP01_OK) {
        CLI_Print(">> WiFi 접속 실패 (SSID/비번 확인 또는 타임아웃)\r\n");
        return;
    }
    CLI_Print(">> WiFi 접속 성공!\r\n");

    HAL_Delay(1000);

    CLI_Print("서버 연결 시도 중...\r\n");
    result = ESP01_ConnectServer(SERVER_IP, SERVER_PORT);
    if (result != ESP01_OK) {
        CLI_Print(">> 서버 연결 실패 (IP/포트 확인 또는 서버가 안 켜져있음)\r\n");
        return;
    }
    CLI_Print(">> 서버 연결 성공!\r\n");

}


static void CLI_PrintMenu(void)
{
    CLI_Print("\r\n========================================\r\n");
    CLI_Print("             CLI 테스트\r\n");
    CLI_Print("========================================\r\n");
    CLI_Print("  0. 통신 확인 (핸드셰이크)\r\n");
    CLI_Print("  1. 지문 등록\r\n");
    CLI_Print("  2. 지문 인식\r\n");
    CLI_Print("  3. 관리자 인증\r\n");
    CLI_Print("  4. WiFi 연결 테스트\r\n");
    CLI_Print("----------------------------------------\r\n");
    CLI_Print("선택: ");
}

/* ------------- 외부 공개 함수 ------------- */

void CLI_Init(void)
{
    CLI_Print("\r\nCLI 시작.\r\n");
}

void CLI_Run(void)
{
    char inputBuf[8];

    CLI_PrintMenu();
    CLI_ReadLine(inputBuf, sizeof(inputBuf));

    switch (inputBuf[0]) {
        case '0':
            CLI_MenuHandshake();
            break;
        case '1':
            CLI_MenuEnroll();
            break;
        case '2':
            CLI_MenuVerify();
            break;
        case '3':
            CLI_MenuAdminAuth();
            break;
        case '4':
            CLI_MenuWifiTest();
            break;
        default:
            CLI_Print("\r\n잘못된 입력입니다.\r\n");
            break;
    }
}