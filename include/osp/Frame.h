/*
 * Copyright (c) 2024, Robert Bosch GmbH
 */

/**
 * @file Frame format for OSP (Open System Protocol) frames by ASM OSRAM.
 * The OSP protocol is not part of the IEEE 1722 spec, but might be transported
 * over IEEE 1722's generic byte bus (GBB).
 * 
 * Example how to build an OSP frame:
 * 
 * @code{.c}
 * uint8_t buf[OSP_MAX_FRAME_LEN] = {0};
 * Osp_Frame_t* frame = (Osp_Frame_t*)buf;
 * 
 * Osp_Frame_Init(frame);
 * Osp_Frame_SetAddress(frame, OSP_ADDRESS_RANGE_BEGIN);
 * Osp_Frame_SetCommand(frame, OSP_COMMAND_INIT);
 * Osp_Frame_SetPayloadLength(frame, 0);
 * Osp_Frame_UpdateCrc(frame);
 * @endcode
 * 
 * Example how to parse an OSP frame
 * 
 * @code{.c}
 * uint8_t buf[OSP_MAX_FRAME_LEN] = {0xA0, 0x04, 0x02, ...};
 * Osp_Frame_t* frame = (Osp_Frame_t*)buf;
 * 
 * // Parse fields
 * uint8_t preamble = Osp_Frame_GetPreamble(frame);
 * uint16_t address = Osp_Frame_GetAddress(frame);
 * uint8_t psi = Osp_Frame_GetPsi(frame);
 * uint8_t cmd = Osp_Frame_GetCommand(frame);
 * uint8_t crc = Osp_Frame_GetCrc(frame);
 * 
 * // Other useful functions
 * uint8_t frameLength = Osp_Frame_GetLength(frame);
 * uint8_t payloadLength = Osp_Frame_GetPayloadLength(frame);
 * if (!Osp_Frame_IsCrcValid(frame)) {
 *     printf("Checksum not valid!");
 * }
 * @endcode
 * 
 */

#pragma once

#include <stdint.h>

/* OSP frame size specific */
#define OSP_HEADER_LEN              3
#define OSP_TRAILER_LEN             1
#define OSP_MAX_FRAME_LEN           12

/* OSP predefined addresses */
#define OSP_ADDRESS_BROADCAST       0x000
#define OSP_ADDRESS_INIT            0x3FF
#define OSP_ADDRESS_RANGE_BEGIN     0x1
#define OSP_ADDRESS_RANGE_END       0x3EF

/* OSP commands */
#define OSP_COMMAND_RESET           0x0
#define OSP_COMMAND_CLEAREVENT      0x1
#define OSP_COMMAND_INIT            0x2
#define OSP_COMMAND_INITLOOP        0x3
#define OSP_COMMAND_IDENTIFY        0x7
#define OSP_COMMAND_PING4EVENT      0x8

/* OSP checksum specific */
#define OSP_CRC_POLYNOMIAL          0x97
#define OSP_CRC_INIT                0x0
#define OSP_CRC_FINAL_XOR           0x0

/* Other OSP constants */
#define OSP_PREAMBLE                0b1010

typedef struct {
    uint8_t header[OSP_HEADER_LEN];
    uint8_t payload[0];
} Osp_Frame_t;

void Osp_Frame_Init(Osp_Frame_t* frame);

uint8_t Osp_Frame_GetPreamble(Osp_Frame_t* frame);
uint16_t Osp_Frame_GetAddress(Osp_Frame_t* frame);
uint8_t Osp_Frame_GetPsi(Osp_Frame_t* frame);
uint8_t Osp_Frame_GetCommand(Osp_Frame_t* frame);
uint8_t Osp_Frame_GetCrc(Osp_Frame_t* frame);
uint8_t* Osp_Frame_GetPayload(Osp_Frame_t* frame);
uint8_t Osp_Frame_GetPayloadLength(Osp_Frame_t* frame);
uint8_t Osp_Frame_GetLength(Osp_Frame_t* frame);

void Osp_Frame_SetPreamble(Osp_Frame_t* frame, uint8_t preamble);
void Osp_Frame_SetAddress(Osp_Frame_t* frame, uint16_t address);
void Osp_Frame_SetPsi(Osp_Frame_t* frame, uint8_t psi);
void Osp_Frame_SetCommand(Osp_Frame_t* frame, uint8_t command);
void Osp_Frame_SetCrc(Osp_Frame_t* frame, uint8_t crc);
void Osp_Frame_SetPayloadLength(Osp_Frame_t* frame, uint8_t length);

uint8_t Osp_Frame_ComputeCrc(Osp_Frame_t* frame);
void Osp_Frame_UpdateCrc(Osp_Frame_t* frame);
uint8_t Osp_Frame_IsCrcValid(Osp_Frame_t* frame);
