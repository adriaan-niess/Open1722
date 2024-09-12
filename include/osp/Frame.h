/*
 * Copyright (c) 2024, Robert Bosch GmbH
 */

/**
 * @file Frame format for OSP (Open System Protocol) frames by ASM OSRAM.
 * The OSP protocol is not part of the IEEE 1722 spec, but might be transported
 * over IEEE 1722's generic byte bus (GBB).
 */

#pragma once

#include <stdint.h>

#define OSP_HEADER_LEN          3
#define OSP_TRAILER_LEN         (OSP_CRC_LEN)
#define OSP_MAX_FRAME_LEN       12
#define OSP_CRC_LEN             1
#define OSP_PREAMBLE            0b1010
#define OSP_BROADCAST_ADDRESS   0x000
#define OSP_INIT_ADDRESS        0x3FF  

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
