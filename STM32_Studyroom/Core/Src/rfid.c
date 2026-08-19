/*
 * rfid.c
 *
 * MFRC522(RC522) RFID 리더 드라이버 구현 (SPI2 + HAL)
 */

#include "rfid.h"
#include "spi.h"
#include <string.h>

extern SPI_HandleTypeDef hspi2;
#define RFID_SPI   (&hspi2)

/* ---------------- MFRC522 레지스터 주소 ---------------- */
#define CommandReg       0x01
#define ComIEnReg        0x02
#define DivIEnReg        0x03
#define ComIrqReg        0x04
#define DivIrqReg        0x05
#define ErrorReg         0x06
#define FIFODataReg      0x09
#define FIFOLevelReg     0x0A
#define BitFramingReg    0x0D
#define CollReg          0x0E
#define ModeReg          0x11
#define TxControlReg     0x14
#define TxASKReg         0x15
#define TModeReg         0x2A
#define TPrescalerReg    0x2B
#define TReloadRegL      0x2C
#define TReloadRegH      0x2D

/* ---------------- MFRC522 커맨드 ---------------- */
#define PCD_IDLE         0x00
#define PCD_TRANSCEIVE   0x0C
#define PCD_RESETPHASE   0x0F

/* ---------------- PICC(카드) 커맨드 ---------------- */
#define PICC_REQIDL      0x26
#define PICC_ANTICOLL    0x93

const uint8_t ADMIN_CARD_UID[RFID_UID_LEN] = { 0xC0, 0x6D, 0x31, 0x5F }; // TODO: 실제 카드 UID로 교체

/* ---------------- 저수준 SPI 레지스터 접근 ---------------- */

static void RFID_CS_Low(void)  { HAL_GPIO_WritePin(RFID_CS_GPIO_Port, RFID_CS_Pin, GPIO_PIN_RESET); }
static void RFID_CS_High(void) { HAL_GPIO_WritePin(RFID_CS_GPIO_Port, RFID_CS_Pin, GPIO_PIN_SET); }

static void RFID_WriteReg(uint8_t addr, uint8_t val)
{
    uint8_t tx[2];
    tx[0] = (addr << 1) & 0x7E; // MSB=0(쓰기), 하위 6비트 주소
    tx[1] = val;

    RFID_CS_Low();
    HAL_SPI_Transmit(RFID_SPI, tx, 2, HAL_MAX_DELAY);
    RFID_CS_High();
}

static uint8_t RFID_ReadReg(uint8_t addr)
{
    uint8_t tx = ((addr << 1) & 0x7E) | 0x80; // MSB=1(읽기)
    uint8_t rx = 0;

    RFID_CS_Low();
    HAL_SPI_Transmit(RFID_SPI, &tx, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(RFID_SPI, &rx, 1, HAL_MAX_DELAY);
    RFID_CS_High();

    return rx;
}

static void RFID_SetBitMask(uint8_t addr, uint8_t mask)
{
    uint8_t tmp = RFID_ReadReg(addr);
    RFID_WriteReg(addr, tmp | mask);
}

static void RFID_ClearBitMask(uint8_t addr, uint8_t mask)
{
    uint8_t tmp = RFID_ReadReg(addr);
    RFID_WriteReg(addr, tmp & (~mask));
}

static void RFID_AntennaOn(void)
{
    uint8_t tmp = RFID_ReadReg(TxControlReg);
    if (!(tmp & 0x03)) {
        RFID_SetBitMask(TxControlReg, 0x03);
    }
}

/* ---------------- 카드와 통신 (송수신 공통 루틴) ---------------- */

static uint8_t RFID_ToCard(uint8_t command, uint8_t *sendData, uint8_t sendLen,
                            uint8_t *backData, uint16_t *backLen)
{
    uint8_t status = MI_ERR;
    uint8_t irqEn = 0x77;
    uint8_t waitIRq = 0x30;
    uint8_t n;
    uint32_t i;

    RFID_WriteReg(ComIEnReg, irqEn | 0x80);
    RFID_ClearBitMask(ComIrqReg, 0x80);
    RFID_SetBitMask(FIFOLevelReg, 0x80); // FIFO 비우기
    RFID_WriteReg(CommandReg, PCD_IDLE);

    for (i = 0; i < sendLen; i++) {
        RFID_WriteReg(FIFODataReg, sendData[i]);
    }

    RFID_WriteReg(CommandReg, command);
    if (command == PCD_TRANSCEIVE) {
        RFID_SetBitMask(BitFramingReg, 0x80); // StartSend
    }

    // 응답 대기 (타임아웃 카운터 기반)
    i = 2000;
    do {
        n = RFID_ReadReg(ComIrqReg);
        i--;
    } while ((i != 0) && !(n & 0x01) && !(n & waitIRq));

    RFID_ClearBitMask(BitFramingReg, 0x80);

    if (i != 0) {
        if (!(RFID_ReadReg(ErrorReg) & 0x1B)) {
            status = MI_OK;
            if (n & irqEn & 0x01) {
                status = MI_NOTAGERR;
            }

            if (command == PCD_TRANSCEIVE) {
                n = RFID_ReadReg(FIFOLevelReg);
                if (n == 0) n = 1;
                if (n > 16) n = 16;

                if (backData != NULL) {
                    for (i = 0; i < n; i++) {
                        backData[i] = RFID_ReadReg(FIFODataReg);
                    }
                }
                if (backLen != NULL) {
                    *backLen = n * 8;
                }
            }
        } else {
            status = MI_ERR;
        }
    }

    return status;
}

/* ---------------- 외부 공개 함수 ---------------- */

void RFID_Reset(void)
{
    RFID_WriteReg(CommandReg, PCD_RESETPHASE);
}

void RFID_Init(void)
{
    // 하드웨어 리셋 (RST 핀 토글)
    HAL_GPIO_WritePin(RFID_RST_GPIO_Port, RFID_RST_Pin, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(RFID_RST_GPIO_Port, RFID_RST_Pin, GPIO_PIN_SET);
    HAL_Delay(50);

    RFID_Reset(); // 소프트 리셋 커맨드

    RFID_WriteReg(TModeReg, 0x8D);
    RFID_WriteReg(TPrescalerReg, 0x3E);
    RFID_WriteReg(TReloadRegL, 30);
    RFID_WriteReg(TReloadRegH, 0);
    RFID_WriteReg(TxASKReg, 0x40);
    RFID_WriteReg(ModeReg, 0x3D);

    RFID_AntennaOn();
}

uint8_t RFID_Request(uint8_t reqMode, uint8_t *tagType)
{
    uint8_t status;
    uint16_t backBits;
    uint8_t sendData[1];

    RFID_WriteReg(BitFramingReg, 0x07);

    sendData[0] = reqMode;
    status = RFID_ToCard(PCD_TRANSCEIVE, sendData, 1, tagType, &backBits);

    if ((status != MI_OK) || (backBits != 0x10)) {
        status = MI_ERR;
    }

    return status;
}

uint8_t RFID_Anticoll(uint8_t *serNum)
{
    uint8_t status;
    uint8_t serNumCheck = 0;
    uint16_t unLen;
    uint8_t i;

    RFID_WriteReg(BitFramingReg, 0x00);

    serNum[0] = PICC_ANTICOLL;
    serNum[1] = 0x20;

    status = RFID_ToCard(PCD_TRANSCEIVE, serNum, 2, serNum, &unLen);

    if (status == MI_OK) {
        // 체크섬 검증: UID 4바이트 XOR == 5번째 바이트
        for (i = 0; i < 4; i++) {
            serNumCheck ^= serNum[i];
        }
        if (serNumCheck != serNum[4]) {
            status = MI_ERR;
        }
    }

    return status;
}

uint8_t RFID_IsAdminCard(const uint8_t *serNum)
{
    return (memcmp(serNum, ADMIN_CARD_UID, RFID_UID_LEN) == 0) ? 1 : 0;
}

uint8_t RFID_ScanCard(uint8_t *uidOut)
{
    uint8_t status;
    uint8_t tagType[2];
    uint8_t serNum[5];

    status = RFID_Request(PICC_REQIDL, tagType);
    if (status != MI_OK) {
        return MI_NOTAGERR; // 카드 없음
    }

    status = RFID_Anticoll(serNum);
    if (status != MI_OK) {
        return MI_ERR;
    }

    memcpy(uidOut, serNum, RFID_UID_LEN);
    return MI_OK;
}