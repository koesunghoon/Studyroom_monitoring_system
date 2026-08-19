/*
 * rfid.h
 *
 * MFRC522(RC522) RFID 리더 드라이버 (SPI, HAL 기반)
 * SPI2 사용 (SCK=PB10, MISO=PC2, MOSI=PC3), SDA(CS)=PB6, RST=PB4
 */

#ifndef RFID_H
#define RFID_H

#include "main.h"
#include <stdint.h>

/* ---------------- 핀 설정 ---------------- */
#define RFID_CS_GPIO_Port    GPIOB
#define RFID_CS_Pin           GPIO_PIN_6

#define RFID_RST_GPIO_Port    GPIOB
#define RFID_RST_Pin           GPIO_PIN_4

/* ---------------- 상태 코드 ---------------- */
#define MI_OK          0
#define MI_NOTAGERR    1
#define MI_ERR         2

/* ---------------- UID 길이 ---------------- */
#define RFID_UID_LEN   4   // 일반 Mifare Classic 4바이트 UID 기준

/* ---------------- 관리자 카드 UID (스캔 후 실제 값으로 교체) ---------------- */
extern const uint8_t ADMIN_CARD_UID[RFID_UID_LEN];

/* ---------------- 함수 프로토타입 ---------------- */
void    RFID_Init(void);
void    RFID_Reset(void);
uint8_t RFID_Request(uint8_t reqMode, uint8_t *tagType);
uint8_t RFID_Anticoll(uint8_t *serNum);

// serNum(4바이트)이 ADMIN_CARD_UID와 일치하는지 확인
uint8_t RFID_IsAdminCard(const uint8_t *serNum);

// 카드 스캔 -> UID 읽기까지 한 번에 수행
// 성공 시 uidOut(4바이트)에 스캔된 UID 채움, 리턴값: MI_OK / MI_ERR / MI_NOTAGERR
uint8_t RFID_ScanCard(uint8_t *uidOut);

#endif /* RFID_H */