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
 * @file PDU format for the Generic Byte Bus (GBB) ACF message.
 */

#pragma once

#include <stdint.h>

#include "avtp/Defines.h"
#include "avtp/acf/AcfCommon.h"

#define AVTP_GBB_HEADER_LEN         (4 * AVTP_QUADLET_SIZE)

typedef struct {
    uint8_t header[AVTP_GBB_HEADER_LEN];
    uint8_t payload[0];
} Avtp_Gbb_t;

typedef enum {
    /* ACF common header fields */
    AVTP_GBB_FIELD_ACF_MSG_TYPE = 0,
    AVTP_GBB_FIELD_ACF_MSG_LENGTH,
    /* ACF GBB header fields */
    AVTP_GBB_FIELD_PAD,
    AVTP_GBB_FIELD_MTV,
    AVTP_GBB_FIELD_RESERVED,
    AVTP_GBB_FIELD_BYTE_BUS_ID,
    AVTP_GBB_FIELD_MESSAGE_TIMESTAMP,
    AVTP_GBB_FIELD_EVT,
    AVTP_GBB_FIELD_RESERVED_2,
    AVTP_GBB_FIELD_HS,
    AVTP_GBB_FIELD_CS,
    AVTP_GBB_FIELD_TRANSACTION_NUM,
    AVTP_GBB_FIELD_OP,
    AVTP_GBB_FIELD_RSP,
    AVTP_GBB_FIELD_ERR,
    AVTP_GBB_FIELD_MS,
    AVTP_GBB_FIELD_SEGMENT_NUM,
    AVTP_GBB_FIELD_READ_SIZE,
    /* Count number of fields for bound checks */
    AVTP_GBB_FIELD_MAX
} Avtp_GbbFields_t;

/**
 * Initializes an ACF GBB PDU header as specified in the IEEE 1722 Specification.
 *
 * @param pdu Pointer to the first bit of a 1722 ACF GBB PDU.
 */
void Avtp_Gbb_Init(Avtp_Gbb_t* pdu);

/**
 * Returns the value of an an ACF GBB PDU field as specified in the IEEE 1722 Specification.
 *
 * @param pdu Pointer to the first bit of an 1722 ACF GBB PDU.
 * @param field Specifies the position of the data field to be read
 * @returns Value of the PDU field.
 */
uint64_t Avtp_Gbb_GetField(Avtp_Gbb_t* pdu, Avtp_GbbFields_t field);

uint8_t Avtp_Gbb_GetAcfMsgType(Avtp_Gbb_t* pdu);
uint16_t Avtp_Gbb_GetAcfMsgLength(Avtp_Gbb_t* pdu);
uint8_t Avtp_Gbb_GetPad(Avtp_Gbb_t* pdu);
uint8_t Avtp_Gbb_GetMtv(Avtp_Gbb_t* pdu);
uint16_t Avtp_Gbb_GetByteBusId(Avtp_Gbb_t* pdu);
uint64_t Avtp_Gbb_GetMessageTimestamp(Avtp_Gbb_t* pdu);
uint8_t Avtp_Gbb_GetEvt(Avtp_Gbb_t* pdu);
uint8_t Avtp_Gbb_GetHs(Avtp_Gbb_t* pdu);
uint8_t Avtp_Gbb_GetCs(Avtp_Gbb_t* pdu);
uint8_t Avtp_Gbb_GetTransactionNum(Avtp_Gbb_t* pdu);
uint8_t Avtp_Gbb_GetOp(Avtp_Gbb_t* pdu);
uint8_t Avtp_Gbb_GetRsp(Avtp_Gbb_t* pdu);
uint8_t Avtp_Gbb_GetErr(Avtp_Gbb_t* pdu);
uint8_t Avtp_Gbb_GetMs(Avtp_Gbb_t* pdu);
uint16_t Avtp_Gbb_GetSegmentNum(Avtp_Gbb_t* pdu);
uint16_t Avtp_Gbb_GetReadSize(Avtp_Gbb_t* pdu);

/**
 * Sets the value of an an ACF GBB PDU field as specified in the IEEE 1722 Specification.
 *
 * @param pdu Pointer to the first bit of an 1722 ACF GBB PDU.
 * @param field Specifies the position of the data field to be read
 * @param value Pointer to location to store the value.
 */
void Avtp_Gbb_SetField(Avtp_Gbb_t* pdu, Avtp_GbbFields_t field, uint64_t value);

void Avtp_Gbb_SetAcfMsgType(Avtp_Gbb_t* pdu, uint8_t value);
void Avtp_Gbb_SetAcfMsgLength(Avtp_Gbb_t* pdu, uint16_t value);
void Avtp_Gbb_SetPad(Avtp_Gbb_t* pdu, uint8_t value);
void Avtp_Gbb_SetMtv(Avtp_Gbb_t* pdu, uint8_t value);
void Avtp_Gbb_SetByteBusId(Avtp_Gbb_t* pdu, uint16_t value);
void Avtp_Gbb_SetMessageTimestamp(Avtp_Gbb_t* pdu, uint64_t value);
void Avtp_Gbb_SetEvt(Avtp_Gbb_t* pdu, uint8_t value);
void Avtp_Gbb_SetHs(Avtp_Gbb_t* pdu, uint8_t value);
void Avtp_Gbb_SetCs(Avtp_Gbb_t* pdu, uint8_t value);
void Avtp_Gbb_SetTransactionNum(Avtp_Gbb_t* pdu, uint8_t value);
void Avtp_Gbb_SetOp(Avtp_Gbb_t* pdu, uint8_t value);
void Avtp_Gbb_SetRsp(Avtp_Gbb_t* pdu, uint8_t value);
void Avtp_Gbb_SetErr(Avtp_Gbb_t* pdu, uint8_t value);
void Avtp_Gbb_SetMs(Avtp_Gbb_t* pdu, uint8_t value);
void Avtp_Gbb_SetSegmentNum(Avtp_Gbb_t* pdu, uint16_t value);
void Avtp_Gbb_SetReadSize(Avtp_Gbb_t* pdu, uint16_t value);
