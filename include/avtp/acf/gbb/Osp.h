/*
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

/**
 * @file PDU format for OSP (Open System Protocol) frames by ASM OSRAM. The OSP
 * protocol is not part of the IEEE 1722 spec, but might be transported over
 * IEEE 1722's generic byte bus (GBB).
 */

#pragma once

#include <stdint.h>

#include "avtp/Defines.h"
#include "avtp/acf/AcfCommon.h"

#define AVTP_OSP_HEADER_LEN     3
#define AVTP_OSP_CRC_LEN        1
#define AVTP_OSP_TRAILER_LEN    (AVTP_OSP_CRC_LEN)
#define AVTP_OSP_PREAMBLE       0b1010

typedef struct {
    uint8_t header[AVTP_OSP_HEADER_LEN + AVTP_OSP_TRAILER_LEN];
    uint8_t payload[0];
} Avtp_Osp_t;

void Avtp_Osp_Init(Avtp_Osp_t* osp);

uint8_t Avtp_Osp_GetPreample(Avtp_Osp_t* osp);
uint16_t Avtp_Osp_GetAddress(Avtp_Osp_t* osp);
uint8_t Avtp_Osp_GetPsi(Avtp_Osp_t* osp);
uint8_t Avtp_Osp_GetCommand(Avtp_Osp_t* osp);
uint8_t Avtp_Osp_GetCrc(Avtp_Osp_t* osp);
uint8_t* Avtp_Osp_GetPayload(Avtp_Osp_t* osp);
uint8_t Avtp_Osp_GetPayloadLength(Avtp_Osp_t* osp);

void Avtp_Osp_SetPreample(Avtp_Osp_t* osp, uint8_t preamble);
void Avtp_Osp_SetAddress(Avtp_Osp_t* osp, uint16_t address);
void Avtp_Osp_SetPsi(Avtp_Osp_t* osp, uint8_t psi);
void Avtp_Osp_SetCommand(Avtp_Osp_t* osp, uint8_t command);
void Avtp_Osp_SetCrc(Avtp_Osp_t* osp, uint8_t crc);
void Avtp_Osp_SetPayloadLength(Avtp_Osp_t* osp, uint8_t length);
