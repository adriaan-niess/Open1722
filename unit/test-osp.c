/*
 * Copyright (c) 2019, Intel Corporation
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
 *    * Neither the name of Intel Corporation nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
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
 */

#include <alloca.h>
#include <stdarg.h>
#include <stddef.h>
#include <string.h>
#include <setjmp.h>
#include <cmocka.h>
#include <errno.h>
#include <stdio.h>

#include "osp/Frame.h"

static void Test_Osp_GetPreamble(void **state)
{
    uint8_t buf[OSP_MAX_FRAME_LEN] = {0x90, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};
    Osp_Frame_t* frame = (Osp_Frame_t*)buf;
    assert_true(Osp_Frame_GetPreamble(frame) == 0x9);
}

static void Test_Osp_GetAddress(void **state)
{
    uint8_t buf[OSP_MAX_FRAME_LEN] = {0x09, 0x84, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};
    Osp_Frame_t* frame = (Osp_Frame_t*)buf;
    assert_true(Osp_Frame_GetAddress(frame) == 0x261);
}

static void Test_Osp_GetPsi(void **state)
{
    uint8_t buf[OSP_MAX_FRAME_LEN] = {0x0, 0x3, 0x80, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};
    Osp_Frame_t* frame = (Osp_Frame_t*)buf;
    assert_true(Osp_Frame_GetPsi(frame) == 0x7);
}

static void Test_Osp_GetCommand(void **state)
{
    uint8_t buf[OSP_MAX_FRAME_LEN] = {0x0, 0x0, 0xC1, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};
    Osp_Frame_t* frame = (Osp_Frame_t*)buf;
    assert_true(Osp_Frame_GetCommand(frame) == 0x41);
}

/** Test Getter for CRC with payload length of zero. */
static void Test_Osp_GetCrc_Length0(void **state)
{
    uint8_t buf[OSP_MAX_FRAME_LEN] = {0x0, 0x0, 0x0, 0xFE, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};
    Osp_Frame_t* frame = (Osp_Frame_t*)buf;
    assert_true(Osp_Frame_GetCrc(frame) == 0xFE);
}

/** Test Getter for CRC with payload length of 1 byte. */
static void Test_Osp_GetCrc_Length1(void **state)
{
    uint8_t buf[OSP_MAX_FRAME_LEN] = {0x0, 0x0, 0x80, 0x0, 0xFE, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};
    Osp_Frame_t* frame = (Osp_Frame_t*)buf;
    assert_true(Osp_Frame_GetCrc(frame) == 0xFE);
}

/** Test Getter for CRC with payload length of 2 byte. */
static void Test_Osp_GetCrc_Length2(void **state)
{
    uint8_t buf[OSP_MAX_FRAME_LEN] = {0x0, 0x1, 0x0, 0x0, 0x0, 0xFE, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};
    Osp_Frame_t* frame = (Osp_Frame_t*)buf;
    assert_true(Osp_Frame_GetCrc(frame) == 0xFE);
}

/** Test Getter for CRC with payload length of 3 byte. */
static void Test_Osp_GetCrc_Length3(void **state)
{
    uint8_t buf[OSP_MAX_FRAME_LEN] = {0x0, 0x1, 0x80, 0x0, 0x0, 0x0, 0xFE, 0x0, 0x0, 0x0, 0x0, 0x0};
    Osp_Frame_t* frame = (Osp_Frame_t*)buf;
    assert_true(Osp_Frame_GetCrc(frame) == 0xFE);
}

/** Test Getter for CRC with payload length of 4 byte. */
static void Test_Osp_GetCrc_Length4(void **state)
{
    uint8_t buf[OSP_MAX_FRAME_LEN] = {0x0, 0x2, 0x0, 0x0, 0x0, 0x0, 0x0, 0xFE, 0x0, 0x0, 0x0, 0x0};
    Osp_Frame_t* frame = (Osp_Frame_t*)buf;
    assert_true(Osp_Frame_GetCrc(frame) == 0xFE);
}

/** Test Getter for CRC with payload length of 6 byte. */
static void Test_Osp_GetCrc_Length6(void **state)
{
    uint8_t buf[OSP_MAX_FRAME_LEN] = {0x0, 0x3, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0xFE, 0x0, 0x0};
    Osp_Frame_t* frame = (Osp_Frame_t*)buf;
    assert_true(Osp_Frame_GetCrc(frame) == 0xFE);
}

/** Test Getter for CRC with payload length of 8 byte. */
static void Test_Osp_GetCrc_Length8(void **state)
{
    uint8_t buf[OSP_MAX_FRAME_LEN] = {0x0, 0x3, 0x80, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0xFE};
    Osp_Frame_t* frame = (Osp_Frame_t*)buf;
    assert_true(Osp_Frame_GetCrc(frame) == 0xFE);
}

static void Test_Osp_GetPayload(void **state)
{
    uint8_t buf[OSP_MAX_FRAME_LEN] = {0x0, 0x0, 0x80, 0xFE, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};
    Osp_Frame_t* frame = (Osp_Frame_t*)buf;
    assert_true(*(Osp_Frame_GetPayload(frame)) == 0xFE);
}

/** Test Getter for payload length of zero. */
static void Test_Osp_GetPayloadLength_Length0(void **state)
{
    uint8_t buf[OSP_MAX_FRAME_LEN] = {0x0, 0x0, 0x0, 0xFE, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};
    Osp_Frame_t* frame = (Osp_Frame_t*)buf;
    assert_true(Osp_Frame_GetPayloadLength(frame) == 0);
}

/** Test Getter for payload length of 1 byte. */
static void Test_Osp_GetPayloadLength_Length1(void **state)
{
    uint8_t buf[OSP_MAX_FRAME_LEN] = {0x0, 0x0, 0x80, 0x0, 0xFE, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};
    Osp_Frame_t* frame = (Osp_Frame_t*)buf;
    assert_true(Osp_Frame_GetPayloadLength(frame) == 1);
}

/** Test Getter for payload length of 2 byte. */
static void Test_Osp_GetPayloadLength_Length2(void **state)
{
    uint8_t buf[OSP_MAX_FRAME_LEN] = {0x0, 0x1, 0x0, 0x0, 0x0, 0xFE, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};
    Osp_Frame_t* frame = (Osp_Frame_t*)buf;
    assert_true(Osp_Frame_GetPayloadLength(frame) == 2);
}

/** Test Getter for payload length of 3 byte. */
static void Test_Osp_GetPayloadLength_Length3(void **state)
{
    uint8_t buf[OSP_MAX_FRAME_LEN] = {0x0, 0x1, 0x80, 0x0, 0x0, 0x0, 0xFE, 0x0, 0x0, 0x0, 0x0, 0x0};
    Osp_Frame_t* frame = (Osp_Frame_t*)buf;
    assert_true(Osp_Frame_GetPayloadLength(frame) == 3);
}

/** Test Getter for payload length of 4 byte. */
static void Test_Osp_GetPayloadLength_Length4(void **state)
{
    uint8_t buf[OSP_MAX_FRAME_LEN] = {0x0, 0x2, 0x0, 0x0, 0x0, 0x0, 0x0, 0xFE, 0x0, 0x0, 0x0, 0x0};
    Osp_Frame_t* frame = (Osp_Frame_t*)buf;
    assert_true(Osp_Frame_GetPayloadLength(frame) == 4);
}

/** Test Getter for payload length of 6 byte. */
static void Test_Osp_GetPayloadLength_Length6(void **state)
{
    uint8_t buf[OSP_MAX_FRAME_LEN] = {0x0, 0x3, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0xFE, 0x0, 0x0};
    Osp_Frame_t* frame = (Osp_Frame_t*)buf;
    assert_true(Osp_Frame_GetPayloadLength(frame) == 6);
}

/** Test Getter for payload length of 8 byte. */
static void Test_Osp_GetPayloadLength_Length8(void **state)
{
    uint8_t buf[OSP_MAX_FRAME_LEN] = {0x0, 0x3, 0x80, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0xFE};
    Osp_Frame_t* frame = (Osp_Frame_t*)buf;
    assert_true(Osp_Frame_GetPayloadLength(frame) == 8);
}

static void Test_Osp_SetPreamble(void **state)
{
    uint8_t buf[OSP_MAX_FRAME_LEN] = {0};
    Osp_Frame_t* frame = (Osp_Frame_t*)buf;
    Osp_Frame_SetPreamble(frame, 0x9);

    uint8_t expected[OSP_MAX_FRAME_LEN] = {0x90, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};
    assert_true(memcmp(buf, expected, OSP_MAX_FRAME_LEN) == 0);
}

static void Test_Osp_SetAddress(void **state)
{
    uint8_t buf[OSP_MAX_FRAME_LEN] = {0};
    Osp_Frame_t* frame = (Osp_Frame_t*)buf;
    Osp_Frame_SetAddress(frame, 0x261);

    uint8_t expected[OSP_MAX_FRAME_LEN] = {0x09, 0x84, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};
    assert_true(memcmp(buf, expected, OSP_MAX_FRAME_LEN) == 0);
}

static void Test_Osp_SetPsi(void **state)
{
    uint8_t buf[OSP_MAX_FRAME_LEN] = {0};
    Osp_Frame_t* frame = (Osp_Frame_t*)buf;
    Osp_Frame_SetPsi(frame, 0x7);

    uint8_t expected[OSP_MAX_FRAME_LEN] = {0x0, 0x3, 0x80, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};
    assert_true(memcmp(buf, expected, OSP_MAX_FRAME_LEN) == 0);
}

static void Test_Osp_SetCommand(void **state)
{
    uint8_t buf[OSP_MAX_FRAME_LEN] = {0};
    Osp_Frame_t* frame = (Osp_Frame_t*)buf;
    Osp_Frame_SetCommand(frame, 0x41);

    uint8_t expected[OSP_MAX_FRAME_LEN] = {0x0, 0x0, 0x41, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};
    assert_true(memcmp(buf, expected, OSP_MAX_FRAME_LEN) == 0);
}

static void Test_Osp_SetPayloadLength_Length0(void **state)
{
    uint8_t buf[OSP_MAX_FRAME_LEN] = {0};
    Osp_Frame_t* frame = (Osp_Frame_t*)buf;
    Osp_Frame_SetPayloadLength(frame, 0);

    uint8_t expected[OSP_MAX_FRAME_LEN] = {0x0, 0x0, 0x00, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};
    assert_true(memcmp(buf, expected, OSP_MAX_FRAME_LEN) == 0);
}

static void Test_Osp_SetPayloadLength_Length1(void **state)
{
    uint8_t buf[OSP_MAX_FRAME_LEN] = {0};
    Osp_Frame_t* frame = (Osp_Frame_t*)buf;
    Osp_Frame_SetPayloadLength(frame, 1);

    uint8_t expected[OSP_MAX_FRAME_LEN] = {0x0, 0x0, 0x80, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};
    assert_true(memcmp(buf, expected, OSP_MAX_FRAME_LEN) == 0);
}

static void Test_Osp_SetPayloadLength_Length2(void **state)
{
    uint8_t buf[OSP_MAX_FRAME_LEN] = {0};
    Osp_Frame_t* frame = (Osp_Frame_t*)buf;
    Osp_Frame_SetPayloadLength(frame, 2);

    uint8_t expected[OSP_MAX_FRAME_LEN] = {0x0, 0x01, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};
    assert_true(memcmp(buf, expected, OSP_MAX_FRAME_LEN) == 0);
}

static void Test_Osp_SetPayloadLength_Length3(void **state)
{
    uint8_t buf[OSP_MAX_FRAME_LEN] = {0};
    Osp_Frame_t* frame = (Osp_Frame_t*)buf;
    Osp_Frame_SetPayloadLength(frame, 3);

    uint8_t expected[OSP_MAX_FRAME_LEN] = {0x0, 0x01, 0x80, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};
    assert_true(memcmp(buf, expected, OSP_MAX_FRAME_LEN) == 0);
}

static void Test_Osp_SetPayloadLength_Length4(void **state)
{
    uint8_t buf[OSP_MAX_FRAME_LEN] = {0};
    Osp_Frame_t* frame = (Osp_Frame_t*)buf;
    Osp_Frame_SetPayloadLength(frame, 4);

    uint8_t expected[OSP_MAX_FRAME_LEN] = {0x0, 0x02, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};
    assert_true(memcmp(buf, expected, OSP_MAX_FRAME_LEN) == 0);
}

static void Test_Osp_SetPayloadLength_Length6(void **state)
{
    uint8_t buf[OSP_MAX_FRAME_LEN] = {0};
    Osp_Frame_t* frame = (Osp_Frame_t*)buf;
    Osp_Frame_SetPayloadLength(frame, 6);

    uint8_t expected[OSP_MAX_FRAME_LEN] = {0x0, 0x03, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};
    assert_true(memcmp(buf, expected, OSP_MAX_FRAME_LEN) == 0);
}

static void Test_Osp_SetPayloadLength_Length8(void **state)
{
    uint8_t buf[OSP_MAX_FRAME_LEN] = {0};
    Osp_Frame_t* frame = (Osp_Frame_t*)buf;
    Osp_Frame_SetPayloadLength(frame, 8);

    uint8_t expected[OSP_MAX_FRAME_LEN] = {0x0, 0x03, 0x80, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};
    assert_true(memcmp(buf, expected, OSP_MAX_FRAME_LEN) == 0);
}

static void Test_Osp_SetCrc_Length0(void **state)
{
    uint8_t buf[OSP_MAX_FRAME_LEN] = {0};
    Osp_Frame_t* frame = (Osp_Frame_t*)buf;
    Osp_Frame_SetPayloadLength(frame, 0);
    Osp_Frame_SetCrc(frame, 0xFE);

    uint8_t expected[OSP_MAX_FRAME_LEN] = {0x0, 0x0, 0x0, 0xFE, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};
    assert_true(memcmp(buf, expected, OSP_MAX_FRAME_LEN) == 0);
}

static void Test_Osp_SetCrc_Length1(void **state)
{
    uint8_t buf[OSP_MAX_FRAME_LEN] = {0};
    Osp_Frame_t* frame = (Osp_Frame_t*)buf;
    Osp_Frame_SetPayloadLength(frame, 1);
    Osp_Frame_SetCrc(frame, 0xFE);

    uint8_t expected[OSP_MAX_FRAME_LEN] = {0x0, 0x0, 0x80, 0x0, 0xFE, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};
    assert_true(memcmp(buf, expected, OSP_MAX_FRAME_LEN) == 0);
}

static void Test_Osp_SetCrc_Length2(void **state)
{
    uint8_t buf[OSP_MAX_FRAME_LEN] = {0};
    Osp_Frame_t* frame = (Osp_Frame_t*)buf;
    Osp_Frame_SetPayloadLength(frame, 2);
    Osp_Frame_SetCrc(frame, 0xFE);

    uint8_t expected[OSP_MAX_FRAME_LEN] = {0x0, 0x01, 0x0, 0x0, 0x0, 0xFE, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};
    assert_true(memcmp(buf, expected, OSP_MAX_FRAME_LEN) == 0);
}

static void Test_Osp_SetCrc_Length3(void **state)
{
    uint8_t buf[OSP_MAX_FRAME_LEN] = {0};
    Osp_Frame_t* frame = (Osp_Frame_t*)buf;
    Osp_Frame_SetPayloadLength(frame, 3);
    Osp_Frame_SetCrc(frame, 0xFE);

    uint8_t expected[OSP_MAX_FRAME_LEN] = {0x0, 0x01, 0x80, 0x0, 0x0, 0x0, 0xFE, 0x0, 0x0, 0x0, 0x0, 0x0};
    assert_true(memcmp(buf, expected, OSP_MAX_FRAME_LEN) == 0);
}

static void Test_Osp_SetCrc_Length4(void **state)
{
    uint8_t buf[OSP_MAX_FRAME_LEN] = {0};
    Osp_Frame_t* frame = (Osp_Frame_t*)buf;
    Osp_Frame_SetPayloadLength(frame, 4);
    Osp_Frame_SetCrc(frame, 0xFE);

    uint8_t expected[OSP_MAX_FRAME_LEN] = {0x0, 0x02, 0x0, 0x0, 0x0, 0x0, 0x0, 0xFE, 0x0, 0x0, 0x0, 0x0};
    assert_true(memcmp(buf, expected, OSP_MAX_FRAME_LEN) == 0);
}

static void Test_Osp_SetCrc_Length6(void **state)
{
    uint8_t buf[OSP_MAX_FRAME_LEN] = {0};
    Osp_Frame_t* frame = (Osp_Frame_t*)buf;
    Osp_Frame_SetPayloadLength(frame, 6);
    Osp_Frame_SetCrc(frame, 0xFE);

    uint8_t expected[OSP_MAX_FRAME_LEN] = {0x0, 0x03, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0xFE, 0x0, 0x0};
    assert_true(memcmp(buf, expected, OSP_MAX_FRAME_LEN) == 0);
}

static void Test_Osp_SetCrc_Length8(void **state)
{
    uint8_t buf[OSP_MAX_FRAME_LEN] = {0};
    Osp_Frame_t* frame = (Osp_Frame_t*)buf;
    Osp_Frame_SetPayloadLength(frame, 8);
    Osp_Frame_SetCrc(frame, 0xFE);

    uint8_t expected[OSP_MAX_FRAME_LEN] = {0x0, 0x03, 0x80, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0xFE};
    assert_true(memcmp(buf, expected, OSP_MAX_FRAME_LEN) == 0);
}

int main(void)
{
    const struct CMUnitTest tests[] = {
        cmocka_unit_test(Test_Osp_GetPreamble),
        cmocka_unit_test(Test_Osp_GetAddress),
        cmocka_unit_test(Test_Osp_GetPsi),
        cmocka_unit_test(Test_Osp_GetCommand),
        cmocka_unit_test(Test_Osp_GetCrc_Length0),
        cmocka_unit_test(Test_Osp_GetCrc_Length1),
        cmocka_unit_test(Test_Osp_GetCrc_Length2),
        cmocka_unit_test(Test_Osp_GetCrc_Length3),
        cmocka_unit_test(Test_Osp_GetCrc_Length4),
        cmocka_unit_test(Test_Osp_GetCrc_Length6),
        cmocka_unit_test(Test_Osp_GetCrc_Length8),
        cmocka_unit_test(Test_Osp_GetPayload),
        cmocka_unit_test(Test_Osp_GetPayloadLength_Length0),
        cmocka_unit_test(Test_Osp_GetPayloadLength_Length1),
        cmocka_unit_test(Test_Osp_GetPayloadLength_Length2),
        cmocka_unit_test(Test_Osp_GetPayloadLength_Length3),
        cmocka_unit_test(Test_Osp_GetPayloadLength_Length4),
        cmocka_unit_test(Test_Osp_GetPayloadLength_Length6),
        cmocka_unit_test(Test_Osp_GetPayloadLength_Length8),
        cmocka_unit_test(Test_Osp_SetPreamble),
        cmocka_unit_test(Test_Osp_SetAddress),
        cmocka_unit_test(Test_Osp_SetPsi),
        cmocka_unit_test(Test_Osp_SetCommand),
        cmocka_unit_test(Test_Osp_SetPayloadLength_Length0),
        cmocka_unit_test(Test_Osp_SetPayloadLength_Length1),
        cmocka_unit_test(Test_Osp_SetPayloadLength_Length2),
        cmocka_unit_test(Test_Osp_SetPayloadLength_Length3),
        cmocka_unit_test(Test_Osp_SetPayloadLength_Length4),
        cmocka_unit_test(Test_Osp_SetPayloadLength_Length6),
        cmocka_unit_test(Test_Osp_SetPayloadLength_Length8),
        cmocka_unit_test(Test_Osp_SetCrc_Length0),
        cmocka_unit_test(Test_Osp_SetCrc_Length1),
        cmocka_unit_test(Test_Osp_SetCrc_Length2),
        cmocka_unit_test(Test_Osp_SetCrc_Length3),
        cmocka_unit_test(Test_Osp_SetCrc_Length4),
        cmocka_unit_test(Test_Osp_SetCrc_Length6),
        cmocka_unit_test(Test_Osp_SetCrc_Length8),
    };
    return cmocka_run_group_tests(tests, NULL, NULL);
}
