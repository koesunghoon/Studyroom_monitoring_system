/*
 * fingerprint.c
 *
 * AS608 지문인식 모듈 드라이버 구현
 */

#include "fingerprint.h"
#include "usart.h"   
#include <string.h>

/* USART1 */
extern UART_HandleTypeDef huart1;
#define AS608_UART   (&huart1)

#define AS608_UART_TIMEOUT   1000   // ms, 기본 통신 타임아웃

/* ================= 저수준 패킷 송수신 ================= */

/*
 * AS608_SendPacket
 *  packetID : AS608_PID_COMMAND 등
 *  data     : 명령코드 + 파라미터 (checksum 제외)
 *  len      : data 길이
 *
 *  패킷 구조: [Header 2][Addr 4][PID 1][Length 2][Data...][Checksum 2]
 *  Length = data길이 + checksum(2byte)
 *  Checksum = PID + Length(상위+하위) + data 전부 더한 값의 하위 2바이트
 */
uint8_t AS608_SendPacket(uint8_t packetID, uint8_t *data, uint16_t len)
{
    uint8_t buf[64];
    uint16_t idx = 0;
    uint16_t packetLength = len + 2; // data + checksum(2byte)
    uint16_t checksum;

    if (len > 32) return AS608_ERR_PACKET; // 버퍼 오버플로 방지

    // Header
    buf[idx++] = AS608_HEADER_H;
    buf[idx++] = AS608_HEADER_L;

    // Address (기본값 0xFFFFFFFF)
    buf[idx++] = (uint8_t)(AS608_DEFAULT_ADDR >> 24);
    buf[idx++] = (uint8_t)(AS608_DEFAULT_ADDR >> 16);
    buf[idx++] = (uint8_t)(AS608_DEFAULT_ADDR >> 8);
    buf[idx++] = (uint8_t)(AS608_DEFAULT_ADDR);

    // Packet ID
    buf[idx++] = packetID;

    // Length
    buf[idx++] = (uint8_t)(packetLength >> 8);
    buf[idx++] = (uint8_t)(packetLength & 0xFF);

    // Checksum 계산 시작값 = PID + Length상위 + Length하위
    checksum = packetID + (uint8_t)(packetLength >> 8) + (uint8_t)(packetLength & 0xFF);

    // Data
    for (uint16_t i = 0; i < len; i++) {
        buf[idx++] = data[i];
        checksum += data[i];
    }

    // Checksum
    buf[idx++] = (uint8_t)(checksum >> 8);
    buf[idx++] = (uint8_t)(checksum & 0xFF);

    if (HAL_UART_Transmit(AS608_UART, buf, idx, AS608_UART_TIMEOUT) != HAL_OK) {
        return AS608_ERR_TIMEOUT;
    }

    return AS608_OK;
}

/*
 * AS608_ReceiveAck
 *  응답 패킷을 받아서 ackData(명령코드 응답 바이트들)와 ackLen을 채움
 *  ackData[0] = Confirmation Code
 *  리턴값 = Confirmation Code (성공/실패 여부는 이걸로 판단)
 */
uint8_t AS608_ReceiveAck(uint8_t *ackData, uint16_t *ackLen, uint32_t timeout)
{
    uint8_t header[9]; // Header(2) + Addr(4) + PID(1) + Length(2)
    uint16_t packetLength;
    uint8_t payload[32];

    // 1) 헤더 9바이트 수신
    if (HAL_UART_Receive(AS608_UART, header, 9, timeout) != HAL_OK) {
        return AS608_ERR_TIMEOUT;
    }

    // 헤더 유효성 체크
    if (header[0] != AS608_HEADER_H || header[1] != AS608_HEADER_L) {
        return AS608_ERR_PACKET;
    }

    packetLength = ((uint16_t)header[7] << 8) | header[8];
    if (packetLength < 3 || packetLength > 32) {
        return AS608_ERR_PACKET; // 비정상 길이
    }

    // 2) payload(=confirmation code + parameters) + checksum 수신
    if (HAL_UART_Receive(AS608_UART, payload, packetLength, timeout) != HAL_OK) {
        return AS608_ERR_TIMEOUT;
    }

    // payload[0] = Confirmation Code
    // payload[packetLength-2 : packetLength-1] = Checksum (검증 생략 가능, 필요시 추가)

    if (ackData != NULL) {
        memcpy(ackData, payload, packetLength - 2); // checksum 제외하고 복사
    }
    if (ackLen != NULL) {
        *ackLen = packetLength - 2;
    }

    return payload[0]; // Confirmation Code 리턴
}

/* ================= 개별 명령 함수 ================= */

uint8_t AS608_GetImage(void)
{
    uint8_t cmd[1] = { AS608_CMD_GETIMAGE };
    uint8_t ack[16];
    uint16_t ackLen;

    if (AS608_SendPacket(AS608_PID_COMMAND, cmd, 1) != AS608_OK) {
        return AS608_ERR_TIMEOUT;
    }
    return AS608_ReceiveAck(ack, &ackLen, AS608_UART_TIMEOUT);
}

uint8_t AS608_GenChar(uint8_t bufferID)
{
    uint8_t cmd[2] = { AS608_CMD_GENCHAR, bufferID };
    uint8_t ack[16];
    uint16_t ackLen;

    if (AS608_SendPacket(AS608_PID_COMMAND, cmd, 2) != AS608_OK) {
        return AS608_ERR_TIMEOUT;
    }
    return AS608_ReceiveAck(ack, &ackLen, AS608_UART_TIMEOUT);
}

uint8_t AS608_RegModel(void)
{
    uint8_t cmd[1] = { AS608_CMD_REGMODEL };
    uint8_t ack[16];
    uint16_t ackLen;

    if (AS608_SendPacket(AS608_PID_COMMAND, cmd, 1) != AS608_OK) {
        return AS608_ERR_TIMEOUT;
    }
    return AS608_ReceiveAck(ack, &ackLen, AS608_UART_TIMEOUT);
}

uint8_t AS608_StoreChar(uint8_t bufferID, uint16_t pageID)
{
    uint8_t cmd[4];
    uint8_t ack[16];
    uint16_t ackLen;

    cmd[0] = AS608_CMD_STORE;
    cmd[1] = bufferID;
    cmd[2] = (uint8_t)(pageID >> 8);
    cmd[3] = (uint8_t)(pageID & 0xFF);

    if (AS608_SendPacket(AS608_PID_COMMAND, cmd, 4) != AS608_OK) {
        return AS608_ERR_TIMEOUT;
    }
    return AS608_ReceiveAck(ack, &ackLen, AS608_UART_TIMEOUT);
}

uint8_t AS608_Search(uint8_t bufferID, uint16_t startPage, uint16_t pageNum,
                      uint16_t *matchID, uint16_t *matchScore)
{
    uint8_t cmd[6];
    uint8_t ack[16];
    uint16_t ackLen;
    uint8_t result;

    cmd[0] = AS608_CMD_SEARCH;
    cmd[1] = bufferID;
    cmd[2] = (uint8_t)(startPage >> 8);
    cmd[3] = (uint8_t)(startPage & 0xFF);
    cmd[4] = (uint8_t)(pageNum >> 8);
    cmd[5] = (uint8_t)(pageNum & 0xFF);

    if (AS608_SendPacket(AS608_PID_COMMAND, cmd, 6) != AS608_OK) {
        return AS608_ERR_TIMEOUT;
    }

    result = AS608_ReceiveAck(ack, &ackLen, AS608_UART_TIMEOUT);

    if (result == AS608_OK && ackLen >= 5) {
        // ack[0]=confirmation code, ack[1:2]=PageID, ack[3:4]=MatchScore
        if (matchID != NULL) {
            *matchID = ((uint16_t)ack[1] << 8) | ack[2];
        }
        if (matchScore != NULL) {
            *matchScore = ((uint16_t)ack[3] << 8) | ack[4];
        }
    }

    return result;
}

uint8_t AS608_Empty(void)
{
    uint8_t cmd[1] = { AS608_CMD_EMPTY };
    uint8_t ack[16];
    uint16_t ackLen;

    if (AS608_SendPacket(AS608_PID_COMMAND, cmd, 1) != AS608_OK) {
        return AS608_ERR_TIMEOUT;
    }
    return AS608_ReceiveAck(ack, &ackLen, AS608_UART_TIMEOUT);
}

uint8_t AS608_HandShake(void)
{
    uint8_t cmd[1] = { AS608_CMD_HANDSHAKE };
    uint8_t ack[16];
    uint16_t ackLen;

    if (AS608_SendPacket(AS608_PID_COMMAND, cmd, 1) != AS608_OK) {
        return AS608_ERR_TIMEOUT;
    }
    return AS608_ReceiveAck(ack, &ackLen, AS608_UART_TIMEOUT);
}

/* ================= 대기 헬퍼 ================= */

/*
 * 손가락이 센서에 올라올 때까지 대기 (폴링)
 * timeout_ms = 0 이면 무한 대기
 */
uint8_t AS608_WaitFinger(uint32_t timeout_ms)
{
    uint32_t startTick = HAL_GetTick();
    uint8_t result;

    while (1) {
        result = AS608_GetImage();
        if (result == AS608_OK) {
            return AS608_OK; // 손가락 감지 + 이미지 캡처 성공
        }
        // result == AS608_ERR_NO_FINGER 이면 계속 재시도

        if (timeout_ms != 0 && (HAL_GetTick() - startTick) > timeout_ms) {
            return AS608_ERR_TIMEOUT;
        }
        HAL_Delay(50);
    }
}

/*
 * 손가락이 센서에서 떨어질 때까지 대기 (등록 시 2번째 스캔 전에 필요)
 */
uint8_t AS608_WaitFingerRemoved(uint32_t timeout_ms)
{
    uint32_t startTick = HAL_GetTick();
    uint8_t result;

    while (1) {
        result = AS608_GetImage();
        if (result == AS608_ERR_NO_FINGER) {
            return AS608_OK; // 손가락 없음 확인됨
        }

        if (timeout_ms != 0 && (HAL_GetTick() - startTick) > timeout_ms) {
            return AS608_ERR_TIMEOUT;
        }
        HAL_Delay(50);
    }
}

/* ================= 시나리오 함수 ================= */

/*
 * AS608_Enroll
 *  지문 등록 전체 시퀀스
 *  pageID : 저장할 플래시 라이브러리 슬롯 번호 (0 ~ 모듈 최대 용량-1, 보통 0~299)
 *
 *  절차: 1차 스캔 -> Buffer1 특징추출 -> 손가락 뗄때까지 대기
 *        -> 2차 스캔 -> Buffer2 특징추출 -> RegModel(결합) -> Store(저장)
 *
 *  리턴값: AS608_OK 성공, 그 외는 실패 코드 (어느 단계에서 실패했는지는
 *          UART printf로 각 단계 결과를 따로 찍어보는 걸 추천)
 */
uint8_t AS608_Enroll(uint16_t pageID)
{
    uint8_t result;

    // 1차 지문 스캔
    result = AS608_WaitFinger(10000); // 10초 타임아웃
    if (result != AS608_OK) return result;

    result = AS608_GenChar(1); // Buffer1에 특징 저장
    if (result != AS608_OK) return result;

    // 손가락 떼기 대기
    result = AS608_WaitFingerRemoved(10000);
    if (result != AS608_OK) return result;

    HAL_Delay(200); // 센서 안정화 딜레이

    // 2차 지문 스캔 (같은 손가락 다시 올리기)
    result = AS608_WaitFinger(10000);
    if (result != AS608_OK) return result;

    result = AS608_GenChar(2); // Buffer2에 특징 저장
    if (result != AS608_OK) return result;

    // 두 특징 결합 (RegModel) - 같은 손가락 아니면 여기서 AS608_ERR_COMBINE_FAIL
    result = AS608_RegModel();
    if (result != AS608_OK) return result;

    // 플래시 라이브러리에 저장
    result = AS608_StoreChar(1, pageID); // 결합된 모델은 CharBuffer1에 위치
    if (result != AS608_OK) return result;

    return AS608_OK; // 등록 성공
}

/*
 * AS608_Verify
 *  지문 인식(검색) 시퀀스
 *  matchID    : 매칭된 경우 라이브러리 페이지ID 리턴
 *  matchScore : 매칭 점수 리턴 (높을수록 정확)
 *
 *  절차: 스캔 -> Buffer1 특징추출 -> 전체 라이브러리 Search
 *  라이브러리 용량은 모듈 스펙에 맞게 pageNum 조정 (AS608 보통 300개)
 */
uint8_t AS608_Verify(uint16_t *matchID, uint16_t *matchScore)
{
    uint8_t result;

    result = AS608_WaitFinger(10000);
    if (result != AS608_OK) return result;

    result = AS608_GenChar(1);
    if (result != AS608_OK) return result;

    // startPage=0, pageNum=300 (모듈 최대 등록 용량에 맞게 조정 필요)
    result = AS608_Search(1, 0, 300, matchID, matchScore);
    return result;
}