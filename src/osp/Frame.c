/*
 * Copyright (c) 2024, Robert Bosch GmbH
 * Copyright (c) 2024, COVESA
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *    * Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *    * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *    * Neither the name of COVESA nor the names of its contributors may be 
 *      used to endorse or promote products derived from this software without
 *      specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "osp/Frame.h"

#include <string.h>

void Osp_Frame_Init(Osp_Frame_t* frame)
{
    memset(frame, 0, OSP_HEADER_LEN + OSP_TRAILER_LEN);
    Osp_Frame_SetPreample(frame, OSP_PREAMBLE);
}

uint8_t Osp_Frame_GetPreample(Osp_Frame_t* frame)
{
    return (frame->header[0] & 0xF0) >> 4;
}

uint16_t Osp_Frame_GetAddress(Osp_Frame_t* frame)
{
    return ((uint16_t)(frame->header[0] & 0x0F) << 8)
            | ((frame->header[1] & 0xFC) >> 2);
}

uint8_t Osp_Frame_GetPsi(Osp_Frame_t* frame)
{
    return ((frame->header[1] & 0x03) << 1)
            | ((frame->header[2] & 0x80) >> 7);
}

uint8_t Osp_Frame_GetCommand(Osp_Frame_t* frame)
{
    return frame->header[3] & 0x7F;
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

void Osp_Frame_SetPreample(Osp_Frame_t* frame, uint8_t preamble)
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
