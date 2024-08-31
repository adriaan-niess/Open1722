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

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <inttypes.h>
#include <unistd.h>
#include <argp.h>
#include <linux/if.h>
#include <linux/if_ether.h>
#include <linux/if_packet.h>
#include <poll.h>

#include "common/common.h"
#include "avtp/CommonHeader.h"
#include "avtp/acf/Ntscf.h"
#include "avtp/acf/Tscf.h"
#include "avtp/acf/Gbb.h"

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

    /** Buffer for receiving Ethernet frames. */
    uint8_t rx_buf[sizeof(gbb_avtp_pdu_t) + MAX_GBB_TRANSMIT_SIZE];

    uint8_t async;
} gbb_talker_t;

static struct argp_option argp_options[] = {
    {"dst-addr", 'd', "MACADDR", 0, "Stream Destination MAC address" },
    {"ifname", 'i', "IFNAME", 0, "Network Interface" },
    {"max-transit-time", 'm', "MSEC", 0, "Maximum Transit Time in ms" },
    {"prio", 'p', "NUM", 0, "SO_PRIORITY to be set in socket" },
    {"tnum", ARGP_TRANSACTION_NUM_OPT, "TNUM", 0, "Transaction number to correlate GBB response with request" },
    {"async", ARGP_ASYNC_OPT, NULL, 0, "No blocking wait for response" },
    { 0 }
};

error_t parse_args(int key, char *arg, struct argp_state *argp_state);
void init_gbb_talker(gbb_talker_t* gbb_talker, gbb_talker_cfg_t* cfg);
int gbb_request(gbb_talker_t* gbb_talker, uint8_t* write, uint16_t write_size, uint8_t* read, uint16_t read_size);
int gbb_request_async(gbb_talker_t* gbb_talker, uint8_t* write, uint16_t write_size, uint8_t* read, uint16_t read_size);

error_t parse_args(int key, char *arg, struct argp_state *argp_state)
{
    gbb_talker_cfg_t* cfg = argp_state->input;

    int res;
    switch (key) {
    case 'd':
        res = sscanf(arg, "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx",
                    &cfg->dst_macaddr[0], &cfg->dst_macaddr[1], &cfg->dst_macaddr[2],
                    &cfg->dst_macaddr[3], &cfg->dst_macaddr[4], &cfg->dst_macaddr[5]);
        if (res != 6) {
            fprintf(stderr, "Invalid address\n");
            exit(EXIT_FAILURE);
        }

        break;
    case 'i':
        strncpy(cfg->ifname, arg, sizeof(cfg->ifname) - 1);
        break;
    case 'm':
        cfg->max_transit_time_ns = atoi(arg);
        break;
    case 'p':
        cfg->priority = atoi(arg);
        break;
    case ARGP_TRANSACTION_NUM_OPT:
        cfg->transaction_num = atoi(arg);
        break;
    case ARGP_ASYNC_OPT:
        cfg->async = 1;
        break;
    }
    return 0;
}

void init_gbb_talker(gbb_talker_t* gbb_talker, gbb_talker_cfg_t* cfg)
{
    memset(gbb_talker, 0, sizeof(gbb_talker_t));

    // Copy configuration
    gbb_talker->priority = cfg->priority;
    gbb_talker->max_transit_time_ns = cfg->max_transit_time_ns;
    gbb_talker->timeout_ms = cfg->timeout_ms;
    gbb_talker->request_stream_id = cfg->request_stream_id;
    gbb_talker->response_stream_id = cfg->response_stream_id;
    gbb_talker->byte_bus_id = cfg->byte_bus_id;
    gbb_talker->transaction_num = cfg->transaction_num;
    gbb_talker->async = cfg->async;
    memcpy(gbb_talker->dst_macaddr, cfg->dst_macaddr, ETH_ALEN);
    memcpy(gbb_talker->ifname, cfg->ifname, IFNAMSIZ);

    gbb_talker->seq_num = 0;

    // Check if interface name is not empty
    if (strcmp(gbb_talker->ifname, "") == 0) {
        fprintf(stderr, "No ifname argument was provided (-i, --ifname)\n");
        exit(EXIT_FAILURE);
    }

    // Open socket
    gbb_talker->socket_fd = create_talker_socket(gbb_talker->priority);
    if (gbb_talker->socket_fd < 0) {
        fprintf(stderr, "Failed to open socket!\n");
        exit(EXIT_FAILURE);
    }

    // Set socket address
    int res = setup_socket_address(
            gbb_talker->socket_fd,
            gbb_talker->ifname,
            gbb_talker->dst_macaddr,
            ETH_P_TSN,
            &gbb_talker->dst_socket_addr);
    if (res < 0) {
        fprintf(stderr, "Failed to set up socket address!\n");
        exit(EXIT_FAILURE);
    }

    // Prepare TX buffer
    gbb_avtp_pdu_t* pdu = (gbb_avtp_pdu_t*)gbb_talker->tx_buf;
    Avtp_Ntscf_Init(&pdu->ntscf);
    Avtp_Ntscf_SetStreamId(&pdu->ntscf, gbb_talker->request_stream_id);
    Avtp_Ntscf_SetSv(&pdu->ntscf, 1);
    Avtp_Gbb_Init(&pdu->gbb);
    Avtp_Gbb_SetByteBusId(&pdu->gbb, gbb_talker->byte_bus_id);
    Avtp_Gbb_SetOp(&pdu->gbb, 0);
    Avtp_Gbb_SetTransactionNum(&pdu->gbb, gbb_talker->transaction_num);
}

int gbb_request(gbb_talker_t* gbb_talker, uint8_t* write, uint16_t write_size, uint8_t* read, uint16_t read_size)
{
    // if (gbb_talker->num_free_transaction_ids <= 0) {
    //     fprintf(stderr, "Run out of transaction numbers!\n");
    //     return -EINVAL;
    // }

    if (write_size > MAX_GBB_TRANSMIT_SIZE) {
        fprintf(stderr, "Max GBB transmit size exceeded!\n");
        return -EINVAL;
    }

    uint8_t transaction_num = 42; // TODO obtain valid transaction number
    
    gbb_avtp_pdu_t* pdu = (gbb_avtp_pdu_t*)gbb_talker->tx_buf;
    Avtp_Ntscf_SetSequenceNum(&pdu->ntscf, gbb_talker->seq_num++);
    uint8_t pad = write_size % 4;
    if (pad > 0) {
        pad = 4 - pad;
    }
    size_t gbb_len = AVTP_GBB_HEADER_LEN + write_size + pad;
    Avtp_Ntscf_SetNtscfDataLength(&pdu->ntscf, gbb_len);
    Avtp_Gbb_SetAcfMsgLength(&pdu->gbb, gbb_len / 4);
    Avtp_Gbb_SetPad(&pdu->gbb, pad);
    Avtp_Gbb_SetReadSize(&pdu->gbb, read_size);
    memcpy(pdu->gbb.payload, write, write_size);

    // Send to network
    size_t pdu_len = AVTP_NTSCF_HEADER_LEN + gbb_len;
    int res = sendto(
            gbb_talker->socket_fd,
            pdu,
            pdu_len,
            0,
            (struct sockaddr *)&gbb_talker->dst_socket_addr,
            sizeof(gbb_talker->dst_socket_addr));
    if (res < 0) {
        fprintf(stderr, "Failed to transmit\n");
    }

    return 0;
}

int gbb_request_async(gbb_talker_t* gbb_talker, uint8_t* write, uint16_t write_size, uint8_t* read, uint16_t read_size)
{
    // TODO
}

int main(int argc, char** argv)
{
    // Set static/default parameters
    gbb_talker_cfg_t cfg = {0};
    cfg.timeout_ms = TIMEOUT_MS;
    cfg.request_stream_id = REQUEST_STREAM_ID;
    cfg.response_stream_id = RESPONSE_STREAM_ID;
    cfg.byte_bus_id = BYTE_BUS_ID;
    cfg.async = 0;

    // Parse command line arguments
    struct argp argp = { argp_options, parse_args };
    argp_parse(&argp, argc, argv, 0, NULL, &cfg);

    // Initialize talker app
    gbb_talker_t gbb_talker;
    init_gbb_talker(&gbb_talker, &cfg);

    // Send data
    uint8_t* payload = gbb_talker.tx_buf + sizeof(gbb_avtp_pdu_t);
    ssize_t payload_len = read(STDIN_FILENO, payload, MAX_GBB_TRANSMIT_SIZE);
    if (payload_len >= 0) {
        size_t frame_size = sizeof(gbb_avtp_pdu_t) + payload_len;
        gbb_request(&gbb_talker, gbb_talker.tx_buf, frame_size, NULL, 0);
    }

    // Wait for response
    struct pollfd fds;
    fds.fd = gbb_talker.socket_fd;
    fds.events = POLLIN;
    if (!gbb_talker.async) {
        while (1) {
            if (poll(&fds, 1, gbb_talker.timeout_ms) >= 0) {
                if (fds.revents & POLLIN) {
                    size_t max_len = sizeof(gbb_avtp_pdu_t) + MAX_GBB_TRANSMIT_SIZE;
                    ssize_t pdu_len = read(fds.fd, gbb_talker.rx_buf, max_len);
                    if (pdu_len > 0) {
                        Avtp_CommonHeader_t* common_header = (Avtp_CommonHeader_t*)(gbb_talker.rx_buf);
                        uint8_t subtype = Avtp_CommonHeader_GetSubtype(common_header);
                        if (subtype == AVTP_SUBTYPE_NTSCF) {
                            Avtp_Ntscf_t* ntscf = (Avtp_Ntscf_t*)common_header;
                            uint64_t stream_id = Avtp_Ntscf_GetStreamId(ntscf);
                            if (stream_id == gbb_talker.response_stream_id) {
                                printf("Received response\n");
                                // TODO handle response
                            }
                        }
                    }
                } else {
                    fprintf(stderr, "Response timed out\n");
                    break;
                }
            }
        }
    }

    return EXIT_SUCCESS;
}
