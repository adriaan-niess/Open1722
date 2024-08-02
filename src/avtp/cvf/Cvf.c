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
 *    * Neither the name of COVESA, Intel Corporation nor the names of its
 *      contributors  may be used to endorse or promote products derived from 
 *      this software without specific prior written permission.
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

#include <string.h>
#include <errno.h>

#include "avtp/cvf/Cvf.h"
#include "avtp/Utils.h"
#include "avtp/CommonHeader.h"

static const Avtp_FieldDescriptor_t fieldDescriptors[AVTP_CVF_FIELD_MAX] =
{
    [AVTP_CVF_FIELD_SUBTYPE]            = { .quadlet = 0, .offset = 0, .bits = 8 },
    [AVTP_CVF_FIELD_SV]                 = { .quadlet = 0, .offset = 8, .bits = 1 },
    [AVTP_CVF_FIELD_VERSION]            = { .quadlet = 0, .offset = 9, .bits = 3 },
    [AVTP_CVF_FIELD_MR]                 = { .quadlet = 0, .offset = 12, .bits = 1 },
    [AVTP_CVF_FIELD_RESERVED]           = { .quadlet = 0, .offset = 13, .bits = 2 },
    [AVTP_CVF_FIELD_TV]                 = { .quadlet = 0, .offset = 15, .bits = 1 },
    [AVTP_CVF_FIELD_SEQUENCE_NUM]       = { .quadlet = 0, .offset = 16, .bits = 8 },
    [AVTP_CVF_FIELD_RESERVED_2]         = { .quadlet = 0, .offset = 24, .bits = 7 },
    [AVTP_CVF_FIELD_TU]                 = { .quadlet = 0, .offset = 31, .bits = 1 },
    [AVTP_CVF_FIELD_STREAM_ID]          = { .quadlet = 1, .offset = 0, .bits = 64 },
    [AVTP_CVF_FIELD_AVTP_TIMESTAMP]     = { .quadlet = 3, .offset = 0, .bits = 32 },
    [AVTP_CVF_FIELD_FORMAT]             = { .quadlet = 4, .offset = 0, .bits = 8 },
    [AVTP_CVF_FIELD_FORMAT_SUBTYPE]     = { .quadlet = 4, .offset = 8, .bits = 8 },
    [AVTP_CVF_FIELD_RESERVED_3]         = { .quadlet = 4, .offset = 16, .bits = 16 },
    [AVTP_CVF_FIELD_STREAM_DATA_LENGTH] = { .quadlet = 5, .offset = 0, .bits = 16 },
    [AVTP_CVF_FIELD_RESERVED_4]         = { .quadlet = 5, .offset = 16, .bits = 2 },
    [AVTP_CVF_FIELD_PTV]                = { .quadlet = 5, .offset = 18, .bits = 1 },
    [AVTP_CVF_FIELD_M]                  = { .quadlet = 5, .offset = 19, .bits = 1 },
    [AVTP_CVF_FIELD_EVT]                = { .quadlet = 5, .offset = 20, .bits = 4 },
    [AVTP_CVF_FIELD_RESERVED_5]         = { .quadlet = 5, .offset = 24, .bits = 8 },
};

int Avtp_Cvf_Init(Avtp_Cvf_t* pdu)
{
    if (pdu == NULL) return -EINVAL;

    memset(pdu, 0, sizeof(Avtp_Cvf_t));
    
    int ret;
    ret = Avtp_Cvf_SetField(pdu, AVTP_CVF_FIELD_SUBTYPE, AVTP_SUBTYPE_CVF);
    if (ret != 0) return ret;
    ret = Avtp_Cvf_SetField(pdu, AVTP_CVF_FIELD_FORMAT, AVTP_CVF_FORMAT_RFC);
    if (ret != 0) return ret;
    ret = Avtp_Cvf_SetField(pdu, AVTP_CVF_FIELD_SV, 1);
    if (ret != 0) return ret;
    
    return 0;
}

int Avtp_Cvf_GetField(Avtp_Cvf_t* pdu, Avtp_CvfField_t field, uint64_t* value)
{
    return Avtp_GetField(fieldDescriptors, AVTP_CVF_FIELD_MAX, (uint8_t*)pdu, field, value);
}

int Avtp_Cvf_SetField(Avtp_Cvf_t* pdu, Avtp_CvfField_t field, uint64_t value)
{
    return Avtp_SetField(fieldDescriptors, AVTP_CVF_FIELD_MAX, (uint8_t*)pdu, field, value);
}

/******************************************************************************
 * Legacy API (deprecated)
 *****************************************************************************/

int avtp_cvf_pdu_get(void* pdu, Avtp_CvfField_t field, uint64_t *val)
{
    return Avtp_Cvf_GetField((Avtp_Cvf_t*)pdu, field, val);
}

int avtp_cvf_pdu_set(void* pdu, Avtp_CvfField_t field, uint64_t val)
{
    return Avtp_Cvf_SetField((Avtp_Cvf_t*)pdu, field, val);
}

int avtp_cvf_pdu_init(void* pdu, uint8_t format_subtype)
{
    int ret;
    ret = Avtp_Cvf_Init((Avtp_Cvf_t*)pdu);
    if (ret != 0) return ret;
    ret = Avtp_Cvf_SetField(pdu, AVTP_CVF_FIELD_FORMAT_SUBTYPE, format_subtype);
    if (ret != 0) return ret;
    ret = Avtp_Cvf_SetField(pdu, AVTP_CVF_FIELD_FORMAT, AVTP_CVF_FORMAT_RFC);
    if (ret != 0) return ret;

    return 0;
}
