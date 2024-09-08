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

/* Copyright (c) 2024 Robert Bosch GmbH and its subsidiaries. This program and
 * the accompanying materials are made available under the terms of the Bosch
 * Internal Open Source License v4 which accompanies this distribution, and is
 * available at http://bios.intranet.bosch.com/bioslv4.txt 
*/

#include <errno.h>
#include <string.h>

#include "avtp/acf/AcfCommon.h"
#include "avtp/acf/gbb/Gbb.h"
#include "avtp/Utils.h"
#include "avtp/Defines.h"

#define GET_FIELD(field) \
        (Avtp_GetField(fieldDesc, AVTP_GBB_FIELD_MAX, (uint8_t*)pdu, field))
#define SET_FIELD(field, value) \
        (Avtp_SetField(fieldDesc, AVTP_GBB_FIELD_MAX, (uint8_t*)pdu, field, value))

/**
 * This table maps all IEEE 1722 ACF GBB header fields to a descriptor.
 */
static const Avtp_FieldDescriptor_t fieldDesc[AVTP_GBB_FIELD_MAX] =
{
    /* ACF common header fields */
    [AVTP_GBB_FIELD_ACF_MSG_TYPE]       = { .quadlet = 0, .offset =  0, .bits = 7 },
    [AVTP_GBB_FIELD_ACF_MSG_LENGTH]     = { .quadlet = 0, .offset =  7, .bits = 9 }, 
    /* ACF GBB header fields */         
    [AVTP_GBB_FIELD_PAD]                = { .quadlet = 0, .offset = 16, .bits =   2 },
    [AVTP_GBB_FIELD_MTV]                = { .quadlet = 0, .offset = 18, .bits =   1 },
    [AVTP_GBB_FIELD_RESERVED]           = { .quadlet = 0, .offset = 19, .bits =   2 },
    [AVTP_GBB_FIELD_BYTE_BUS_ID]        = { .quadlet = 0, .offset = 21, .bits =  11 },
    [AVTP_GBB_FIELD_MESSAGE_TIMESTAMP]  = { .quadlet = 1, .offset =  0, .bits =  64 },
    [AVTP_GBB_FIELD_EVT]                = { .quadlet = 3, .offset =  0, .bits =   4 },
    [AVTP_GBB_FIELD_RESERVED_2]         = { .quadlet = 3, .offset =  4, .bits =   2 },
    [AVTP_GBB_FIELD_HS]                 = { .quadlet = 3, .offset =  6, .bits =   1 },
    [AVTP_GBB_FIELD_CS]                 = { .quadlet = 3, .offset =  7, .bits =   1 },
    [AVTP_GBB_FIELD_TRANSACTION_NUM]    = { .quadlet = 3, .offset =  8, .bits =   8 },
    [AVTP_GBB_FIELD_OP]                 = { .quadlet = 3, .offset = 16, .bits =   1 },
    [AVTP_GBB_FIELD_RSP]                = { .quadlet = 3, .offset = 17, .bits =   1 },
    [AVTP_GBB_FIELD_ERR]                = { .quadlet = 3, .offset = 18, .bits =   1 },
    [AVTP_GBB_FIELD_MS]                 = { .quadlet = 3, .offset = 19, .bits =   1 },
    [AVTP_GBB_FIELD_SEGMENT_NUM]        = { .quadlet = 3, .offset = 20, .bits =  12 },
    [AVTP_GBB_FIELD_READ_SIZE]          = { .quadlet = 3, .offset = 20, .bits =  12 },
};

void Avtp_Gbb_Init(Avtp_Gbb_t* pdu)
{
    if(pdu != NULL) {
        memset(pdu, 0, sizeof(Avtp_Gbb_t));  
        Avtp_Gbb_SetField(pdu, AVTP_GBB_FIELD_ACF_MSG_TYPE, AVTP_ACF_TYPE_BYTE_BUS);
    }
}

uint64_t Avtp_Gbb_GetField(Avtp_Gbb_t* pdu, Avtp_GbbFields_t field)
{
    return GET_FIELD(field);
}

uint8_t Avtp_Gbb_GetAcfMsgType(Avtp_Gbb_t* pdu)
{
    return GET_FIELD(AVTP_GBB_FIELD_ACF_MSG_TYPE);
}

uint16_t Avtp_Gbb_GetAcfMsgLength(Avtp_Gbb_t* pdu)
{
    return GET_FIELD(AVTP_GBB_FIELD_ACF_MSG_LENGTH);
}

uint8_t Avtp_Gbb_GetPad(Avtp_Gbb_t* pdu)
{
    return GET_FIELD(AVTP_GBB_FIELD_PAD);
}

uint8_t Avtp_Gbb_GetMtv(Avtp_Gbb_t* pdu)
{
    return GET_FIELD(AVTP_GBB_FIELD_MTV);
}

uint16_t Avtp_Gbb_GetByteBusId(Avtp_Gbb_t* pdu)
{
    return GET_FIELD(AVTP_GBB_FIELD_BYTE_BUS_ID);
}

uint64_t Avtp_Gbb_GetMessageTimestamp(Avtp_Gbb_t* pdu)
{
    return GET_FIELD(AVTP_GBB_FIELD_MESSAGE_TIMESTAMP);
}

uint8_t Avtp_Gbb_GetEvt(Avtp_Gbb_t* pdu)
{
    return GET_FIELD(AVTP_GBB_FIELD_EVT);
}

uint8_t Avtp_Gbb_GetHs(Avtp_Gbb_t* pdu)
{
    return GET_FIELD(AVTP_GBB_FIELD_HS);
}

uint8_t Avtp_Gbb_GetCs(Avtp_Gbb_t* pdu)
{
    return GET_FIELD(AVTP_GBB_FIELD_CS);
}

uint8_t Avtp_Gbb_GetTransactionNum(Avtp_Gbb_t* pdu)
{
    return GET_FIELD(AVTP_GBB_FIELD_TRANSACTION_NUM);
}

uint8_t Avtp_Gbb_GetOp(Avtp_Gbb_t* pdu)
{
    return GET_FIELD(AVTP_GBB_FIELD_OP);
}

uint8_t Avtp_Gbb_GetRsp(Avtp_Gbb_t* pdu)
{
    return GET_FIELD(AVTP_GBB_FIELD_RSP);
}

uint8_t Avtp_Gbb_GetErr(Avtp_Gbb_t* pdu)
{
    return GET_FIELD(AVTP_GBB_FIELD_ERR);
}

uint8_t Avtp_Gbb_GetMs(Avtp_Gbb_t* pdu)
{
    return GET_FIELD(AVTP_GBB_FIELD_MS);
}

uint16_t Avtp_Gbb_GetSegmentNum(Avtp_Gbb_t* pdu)
{
    return GET_FIELD(AVTP_GBB_FIELD_SEGMENT_NUM);
}

uint16_t Avtp_Gbb_GetReadSize(Avtp_Gbb_t* pdu)
{
    return GET_FIELD(AVTP_GBB_FIELD_READ_SIZE);
}

void Avtp_Gbb_SetField(Avtp_Gbb_t* pdu, Avtp_GbbFields_t field, uint64_t value)
{
    SET_FIELD(field, value);
}

void Avtp_Gbb_SetAcfMsgType(Avtp_Gbb_t* pdu, uint8_t value)
{
    SET_FIELD(AVTP_GBB_FIELD_ACF_MSG_TYPE, value);
}

void Avtp_Gbb_SetAcfMsgLength(Avtp_Gbb_t* pdu, uint16_t value)
{
    SET_FIELD(AVTP_GBB_FIELD_ACF_MSG_LENGTH, value);
}

void Avtp_Gbb_SetPad(Avtp_Gbb_t* pdu, uint8_t value)
{
    SET_FIELD(AVTP_GBB_FIELD_PAD, value);
}

void Avtp_Gbb_SetMtv(Avtp_Gbb_t* pdu, uint8_t value)
{
    SET_FIELD(AVTP_GBB_FIELD_MTV, value);
}

void Avtp_Gbb_SetByteBusId(Avtp_Gbb_t* pdu, uint16_t value)
{
    SET_FIELD(AVTP_GBB_FIELD_BYTE_BUS_ID, value);
}

void Avtp_Gbb_SetMessageTimestamp(Avtp_Gbb_t* pdu, uint64_t value)
{
    SET_FIELD(AVTP_GBB_FIELD_MESSAGE_TIMESTAMP, value);
}

void Avtp_Gbb_SetEvt(Avtp_Gbb_t* pdu, uint8_t value)
{
    SET_FIELD(AVTP_GBB_FIELD_EVT, value);
}

void Avtp_Gbb_SetHs(Avtp_Gbb_t* pdu, uint8_t value)
{
    SET_FIELD(AVTP_GBB_FIELD_HS, value);
}

void Avtp_Gbb_SetCs(Avtp_Gbb_t* pdu, uint8_t value)
{
    SET_FIELD(AVTP_GBB_FIELD_CS, value);
}

void Avtp_Gbb_SetTransactionNum(Avtp_Gbb_t* pdu, uint8_t value)
{
    SET_FIELD(AVTP_GBB_FIELD_TRANSACTION_NUM, value);
}

void Avtp_Gbb_SetOp(Avtp_Gbb_t* pdu, uint8_t value)
{
    SET_FIELD(AVTP_GBB_FIELD_OP, value);
}

void Avtp_Gbb_SetRsp(Avtp_Gbb_t* pdu, uint8_t value)
{
    SET_FIELD(AVTP_GBB_FIELD_RSP, value);
}

void Avtp_Gbb_SetErr(Avtp_Gbb_t* pdu, uint8_t value)
{
    SET_FIELD(AVTP_GBB_FIELD_ERR, value);
}

void Avtp_Gbb_SetMs(Avtp_Gbb_t* pdu, uint8_t value)
{
    SET_FIELD(AVTP_GBB_FIELD_MS, value);
}

void Avtp_Gbb_SetSegmentNum(Avtp_Gbb_t* pdu, uint16_t value)
{
    SET_FIELD(AVTP_GBB_FIELD_SEGMENT_NUM, value);
}

void Avtp_Gbb_SetReadSize(Avtp_Gbb_t* pdu, uint16_t value)
{
    SET_FIELD(AVTP_GBB_FIELD_READ_SIZE, value);
}
