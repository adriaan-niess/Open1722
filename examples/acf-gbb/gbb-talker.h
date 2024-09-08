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

#pragma once

#include <stdint.h>
#include <inttypes.h>
#include <argp.h>
#include <linux/if.h>
#include <linux/if_ether.h>
#include <linux/if_packet.h>

#include "common/common.h"
#include "avtp/CommonHeader.h"
#include "avtp/acf/Ntscf.h"
#include "avtp/acf/Tscf.h"
#include "avtp/acf/gbb/Gbb.h"

/**
 * Timeout when waiting for responses.
 */
#define TIMEOUT_MS 1000

/**
 * Stream ID to used for transmitting GBB requests.
 */
#define REQUEST_STREAM_ID 0xAABBCCDDEEFF0001

/**
 * Stream ID used to filter responses.
 */
#define RESPONSE_STREAM_ID 0xAABBCCDDEEFF0002

/**
 * Max size of GBB transmit. Should fit into a single Ethernet frame because
 * segmentation is not supported.
 */
#define MAX_GBB_TRANSMIT_SIZE 1000

/**
 * Default transaction number
 */
#define TRANSACTION_NUM 1

#define BYTE_BUS_ID 55

#define ARGP_TRANSACTION_NUM_OPT 128

#define ARGP_ASYNC_OPT 129

/**
 * AVTP headers for transmitting GBB requests/responses.
 */
typedef struct {
    Avtp_Ntscf_t ntscf;
    Avtp_Gbb_t gbb;
} gbb_avtp_pdu_t;

/**
 * Configuration parameters for GBB talker app.
 */
typedef struct {
    char ifname[IFNAMSIZ];
    int priority;
    uint64_t max_transit_time_ns;
    uint64_t request_stream_id;
    uint64_t response_stream_id;
    uint8_t dst_macaddr[ETH_ALEN];
    int timeout_ms;
    uint16_t byte_bus_id;
    uint8_t transaction_num;
    uint8_t async;
} gbb_talker_cfg_t;

/**
 * State of GBB talker app.
 */
typedef struct {
    char ifname[IFNAMSIZ];
    int priority;
    uint64_t max_transit_time_ns;
    uint64_t request_stream_id;
    uint64_t response_stream_id;
    uint8_t dst_macaddr[ETH_ALEN];

    uint16_t byte_bus_id;

    /** Max time to wait for response */
    int timeout_ms;

    /** Sequence counter. */
    uint8_t seq_num;

    /** Socket file descriptor for sending packets. */
    int socket_fd;

    /** Socket address for sending packets. */
    struct sockaddr_ll dst_socket_addr;

    /** File descriptor from whom the input data is red. */
    int input_fd;

    /** Transaction number for GBB request. */
    uint8_t transaction_num;

    /** Buffer to construct Ethernet frames for transmission */
    uint8_t tx_buf[sizeof(gbb_avtp_pdu_t) + MAX_GBB_TRANSMIT_SIZE];

    size_t tx_buf_level;

    /** Buffer for receiving Ethernet frames. */
    uint8_t rx_buf[sizeof(gbb_avtp_pdu_t) + MAX_GBB_TRANSMIT_SIZE];

    size_t rx_buf_level;

    uint8_t async;
} gbb_talker_t;

error_t parse_args(int key, char *arg, struct argp_state *argp_state);
void gbb_talker_init(gbb_talker_t* gbb_talker, gbb_talker_cfg_t* cfg);
int gbb_talker_send_request(gbb_talker_t* gbb_talker, uint8_t* write, uint16_t write_size, uint8_t* read, uint16_t read_size);
int gbb_talker_send_request_async(gbb_talker_t* gbb_talker, uint8_t* write, uint16_t write_size, uint8_t* read, uint16_t read_size);
int gbb_talker_handle_response(gbb_talker_t* gbb_talker);
int gbb_talker_run(gbb_talker_t* gbb_talker);
