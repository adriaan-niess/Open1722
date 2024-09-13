/*
 * Copyright (c) 2024, Robert Bosch GmbH
 */

#include "osp/Frame.h"

#include <string.h>

void Osp_Frame_Init(Osp_Frame_t* frame)
{
    memset(frame, 0, OSP_HEADER_LEN + OSP_TRAILER_LEN);
    Osp_Frame_SetPreamble(frame, OSP_PREAMBLE);
}

uint8_t Osp_Frame_GetPreamble(Osp_Frame_t* frame)
{
    return (frame->header[0] & 0xF0) >> 4;
}

uint16_t Osp_Frame_GetAddress(Osp_Frame_t* frame)
{
    return ((uint16_t)(frame->header[0] & 0x0F) << 6)
            | ((frame->header[1] & 0xFC) >> 2);
}

uint8_t Osp_Frame_GetPsi(Osp_Frame_t* frame)
{
    return ((frame->header[1] & 0x03) << 1)
            | ((frame->header[2] & 0x80) >> 7);
}

uint8_t Osp_Frame_GetCommand(Osp_Frame_t* frame)
{
    return frame->header[2] & 0x7F;
}

uint8_t Osp_Frame_GetCrc(Osp_Frame_t* frame)
{
    uint8_t payloadLength = Osp_Frame_GetPayloadLength(frame);
    return frame->payload[payloadLength];
}

uint8_t* Osp_Frame_GetPayload(Osp_Frame_t* frame)
{
    return &(frame->payload[0]);
}

uint8_t Osp_Frame_GetPayloadLength(Osp_Frame_t* frame)
{
    uint8_t psi = Osp_Frame_GetPsi(frame);
    uint8_t len = 0;
    if (psi == 0x7) {
        len = 8;
    } else if (psi == 0x5) {
        len = 0;
    } else {
        len = psi;
    }
    return len;
}

uint8_t Osp_Frame_GetLength(Osp_Frame_t* frame)
{
    return OSP_HEADER_LEN + Osp_Frame_GetPayloadLength(frame) + OSP_TRAILER_LEN;
}

void Osp_Frame_SetPreamble(Osp_Frame_t* frame, uint8_t preamble)
{
    frame->header[0] = (~0xF0 & frame->header[0]) | (0xF0 & (preamble << 4));
}

void Osp_Frame_SetAddress(Osp_Frame_t* frame, uint16_t address)
{
    frame->header[0] = (~0x0F & frame->header[0]) | (0x0F & (address >> 6));
    frame->header[1] = (~0xFC & frame->header[1]) | (0xFC & (address << 2));
}

void Osp_Frame_SetPsi(Osp_Frame_t* frame, uint8_t psi)
{
    frame->header[1] = (~0x03 & frame->header[1]) | (0x03 & (psi >> 1));
    frame->header[2] = (~0x80 & frame->header[2]) | (0x80 & (psi << 7));
}

void Osp_Frame_SetCommand(Osp_Frame_t* frame, uint8_t command)
{
    frame->header[2] = (~0x7F & frame->header[2]) | (0x7F & command);
}

void Osp_Frame_SetCrc(Osp_Frame_t* frame, uint8_t crc)
{
    uint8_t payloadLength = Osp_Frame_GetPayloadLength(frame);
    frame->payload[payloadLength] = crc;
}

void Osp_Frame_SetPayloadLength(Osp_Frame_t* frame, uint8_t length)
{
    if (length >= 0x7) {
        Osp_Frame_SetPsi(frame, 0x7);
    } else if (length >= 0x5 && length < 0x7) {
        Osp_Frame_SetPsi(frame, 0x6);
    } else {
        Osp_Frame_SetPsi(frame, length);
    }
}

uint8_t Osp_Frame_ComputeCrc(Osp_Frame_t* frame)
{
    uint8_t len = OSP_HEADER_LEN + Osp_Frame_GetPayloadLength(frame);
    uint8_t* frameBuf = (uint8_t*)frame;
    uint8_t crc = OSP_CRC_INIT;
    size_t i, j;
    for (i = 0; i < len; i++) {
        crc ^= frameBuf[i];
        // TODO inner loop can be replaced with faster lookup table
        for (j = 0; j < 8; j++) {
            if ((crc & 0x80) != 0)
                crc = (uint8_t)((crc << 1) ^ OSP_CRC_POLYNOMIAL);
            else
                crc <<= 1;
        }
    }
    return crc ^ OSP_CRC_FINAL_XOR;
}

void Osp_Frame_UpdateCrc(Osp_Frame_t* frame)
{
    Osp_Frame_SetCrc(frame, Osp_Frame_ComputeCrc(frame));
}

uint8_t Osp_Frame_IsCrcValid(Osp_Frame_t* frame)
{
    return Osp_Frame_GetCrc(frame) == Osp_Frame_ComputeCrc(frame);
}
