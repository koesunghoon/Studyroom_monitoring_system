/*
 * fingerprint.h
 *
 * AS608 지문인식 모듈 드라이버 (UART 통신, HAL 기반)
 * STM32F411RE + USART1 (PA9=TX, PA10=RX) 기준
 *
 * 사용 전 CubeMX에서 USART1 Asynchronous, 57600bps, 8N1 설정 필요
 * usart.c에 extern UART_HandleTypeDef huart1; 이 이미 있어야 함 (CubeMX 자동 생성)
 */

#ifndef FINGERPRINT_H
#define FINGERPRINT_H

#include "main.h"
#include <stdint.h>

/* ---------------- 패킷 상수 ---------------- */
#define AS608_HEADER_H          0xEF
#define AS608_HEADER_L          0x01
#define AS608_DEFAULT_ADDR      0xFFFFFFFF

/* 패킷 식별자 */
#define AS608_PID_COMMAND       0x01
#define AS608_PID_DATA          0x02
#define AS608_PID_ACK           0x07
#define AS608_PID_END_DATA      0x08

/* 명령어 코드 */
#define AS608_CMD_GETIMAGE      0x01
#define AS608_CMD_GENCHAR       0x02
#define AS608_CMD_MATCH         0x03
#define AS608_CMD_SEARCH        0x04
#define AS608_CMD_REGMODEL      0x05
#define AS608_CMD_STORE         0x06
#define AS608_CMD_LOADCHAR      0x07
#define AS608_CMD_DELETECHAR    0x0C
#define AS608_CMD_EMPTY         0x0D
#define AS608_CMD_HANDSHAKE     0x35

/* 응답 확인 코드 (Confirmation Code) */
#define AS608_OK                 0x00  // 성공
#define AS608_ERR_PACKET         0x01  // 패킷 수신 오류
#define AS608_ERR_NO_FINGER      0x02  // 손가락 감지 안됨
#define AS608_ERR_ENROLL_FAIL    0x03  // 이미지 등록 실패
#define AS608_ERR_DISORDER       0x06  // 지문이 너무 지저분함(특징점 생성 실패)
#define AS608_ERR_SMALL_FEATURE  0x07  // 특징점 부족 / 이미지 너무 작음
#define AS608_ERR_NOMATCH        0x08  // 지문 불일치
#define AS608_ERR_NOTFOUND       0x09  // 매칭되는 지문 없음
#define AS608_ERR_COMBINE_FAIL   0x0A  // 두 특징 결합 실패(RegModel 실패, 서로 다른 손가락)
#define AS608_ERR_BAD_LOCATION   0x0B  // 잘못된 페이지ID(라이브러리 범위 초과)
#define AS608_ERR_TIMEOUT        0xFF  // 통신 타임아웃 (자체 정의, AS608 스펙 아님)

/* 함수 프로토타입 */

// 저수준 패킷 송수신
uint8_t AS608_SendPacket(uint8_t packetID, uint8_t *data, uint16_t len);
uint8_t AS608_ReceiveAck(uint8_t *ackData, uint16_t *ackLen, uint32_t timeout);

// 개별 명령
uint8_t AS608_GetImage(void);
uint8_t AS608_GenChar(uint8_t bufferID);
uint8_t AS608_RegModel(void);
uint8_t AS608_StoreChar(uint8_t bufferID, uint16_t pageID);
uint8_t AS608_Search(uint8_t bufferID, uint16_t startPage, uint16_t pageNum,
                      uint16_t *matchID, uint16_t *matchScore);
uint8_t AS608_Empty(void);
uint8_t AS608_HandShake(void);

// 대기 헬퍼
uint8_t AS608_WaitFinger(uint32_t timeout_ms);
uint8_t AS608_WaitFingerRemoved(uint32_t timeout_ms);

// 시나리오 함수 (등록 / 인식)
uint8_t AS608_Enroll(uint16_t pageID);
uint8_t AS608_Verify(uint16_t *matchID, uint16_t *matchScore);

#endif /* FINGERPRINT_H */