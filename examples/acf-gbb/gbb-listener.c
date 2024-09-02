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
#include <unistd.h>
#include <string.h>
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
 * Stream ID to used for transmitting GBB requests.
 */
#define REQUEST_STREAM_ID 0xAABBCCDDEEFF0001

/**
 * Stream ID used to filter responses.
 */
#define RESPONSE_STREAM_ID 0xAABBCCDDEEFF0002

/**
 * Byte Bus ID used to filter ACF messages in request and to use for ACF
 * message(s) in response.
 */
#define BYTE_BUS_ID 55

/**
 * Max header size used for AVTP frames.
 */
#define MAX_AVTP_HEADER_SIZE (AVTP_COMMON_HEADER_LEN + AVTP_TSCF_HEADER_LEN)

/**
 * Max payload to be transported within a GBB frame.
 */
#define MAX_GBB_TRANSMIT_SIZE 1000

/**
 * Size of RX buffer storing AVTP frames with GBB requests.
 */
#define RX_BUFFER_SIZE (MAX_AVTP_HEADER_SIZE + MAX_GBB_TRANSMIT_SIZE)

/**
 * Size of TX buffer storing AVTP frames with GBB responses.
 */
#define TX_BUFFER_SIZE (MAX_AVTP_HEADER_SIZE + MAX_GBB_TRANSMIT_SIZE)

/**
 * Configuration parameters for GBB talker app.
 */
typedef struct {
    char ifname[IFNAMSIZ];
    uint8_t dst_macaddr[ETH_ALEN];
    int priority;
    uint64_t request_stream_id;
    uint64_t response_stream_id;
    uint16_t byte_bus_id;
} gbb_listener_cfg_t;

/**
 * State of GBB talker app.
 */
typedef struct {
    /** Ethernet interface to use in combination with raw socket. */
    char ifname[IFNAMSIZ];

    /** Socket priority */
    int priority;

    /** Stream ID to filter requests */
    uint64_t request_stream_id;

    /** Stream ID used for responses */
    uint64_t response_stream_id;
    
    /** destination MAC address for responses. */
    uint8_t dst_macaddr[ETH_ALEN];

    /**
     * Byte Bus ID used to filter ACF messages. Only GBB/ABB messages matching
     * this byte bus ID are processed by the listener app.
     */
    uint16_t byte_bus_id;

    /** Next expected sequence number for request. */
    uint8_t seq_num_request;

    /** Next sequence number to use for response. */
    uint8_t seq_num_response;

    /** Socket file descriptor for sending packets. */
    int socket_fd;

    /** Socket address for sending packets. */
    struct sockaddr_ll dst_socket_addr;

    /** Buffer for response PDUs. */
    uint8_t tx_buf[TX_BUFFER_SIZE];

    /** Part of TX buffer that contains valid data. */
    uint16_t tx_buf_level;

    /** Buffer for request PDUs. */
    uint8_t rx_buf[RX_BUFFER_SIZE];

    /** Part of RX buffer that contains valid data. */
    uint16_t rx_buf_level;
} gbb_listener_t;

static struct argp_option argp_options[] = {
    {"dst-addr", 'd', "MACADDR", 0, "Stream Destination MAC address" },
    {"ifname", 'i', "IFNAME", 0, "Network Interface" },
    {"prio", 'p', "NUM", 0, "SO_PRIORITY to be set in socket" },
    { 0 }
};

error_t parse_args(int key, char *arg, struct argp_state *argp_state);
void gbb_listener_init(gbb_listener_t* gbb_listener, gbb_listener_cfg_t* cfg);

/**
 * Handle ACF message (GBB request).
 */
void gbb_listener_handle_acf_msg(gbb_listener_t* gbb_listener, Avtp_AcfCommon_t* request);

/**
 * Handle incoming AVTP frames stored in RX buffer.
 */
void gbb_listener_handle_request(gbb_listener_t* gbb_listener);

error_t parse_args(int key, char *arg, struct argp_state *argp_state)
{
    gbb_listener_t* cfg = argp_state->input;

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
    case 'p':
        cfg->priority = atoi(arg);
        break;
    }
    return 0;
}

void gbb_listener_init(gbb_listener_t* gbb_listener, gbb_listener_cfg_t* cfg)
{
    // Zero everything just in case
    memset(gbb_listener, 0, sizeof(gbb_listener_t));

    // External configuration parameters
    memcpy(gbb_listener->ifname, cfg->ifname, IFNAMSIZ);
    gbb_listener->priority = cfg->priority;
    gbb_listener->request_stream_id = cfg->request_stream_id;
    gbb_listener->response_stream_id = cfg->response_stream_id;
    memcpy(gbb_listener->dst_macaddr, cfg->dst_macaddr, ETH_ALEN);
    gbb_listener->byte_bus_id = cfg->byte_bus_id;

    // Static configuration parameters
    gbb_listener->seq_num_request = 0;
    gbb_listener->seq_num_response = 0;
    gbb_listener->tx_buf_level = 0;
    gbb_listener->rx_buf_level = 0;

    // Check if interface name is not empty
    if (strcmp(gbb_listener->ifname, "") == 0) {
        fprintf(stderr, "No ifname argument was provided (-i, --ifname)\n");
        exit(EXIT_FAILURE);
    }

    // Open socket
    gbb_listener->socket_fd = create_talker_socket(gbb_listener->priority);
    if (gbb_listener->socket_fd < 0) {
        fprintf(stderr, "Failed to open socket!\n");
        exit(EXIT_FAILURE);
    }

    // Set socket address
    int res = setup_socket_address(
            gbb_listener->socket_fd,
            gbb_listener->ifname,
            gbb_listener->dst_macaddr,
            ETH_P_TSN,
            &gbb_listener->dst_socket_addr);
    if (res < 0) {
        fprintf(stderr, "Failed to set up socket address!\n");
        exit(EXIT_FAILURE);
    }
}

void gbb_listener_handle_request(gbb_listener_t* gbb_listener)
{
    Avtp_CommonHeader_t* common_header = (Avtp_CommonHeader_t*)(gbb_listener->rx_buf);
    uint8_t subtype = Avtp_CommonHeader_GetSubtype(common_header);

    // Check if frame matches stream ID and uses control format (NTSCF or TSCF)
    uint8_t filter_frame = 1;
    Avtp_AcfCommon_t* first_acf_msg = NULL;
    size_t acf_list_size = 0;
    uint64_t stream_id = 0;
    if (subtype == AVTP_SUBTYPE_NTSCF) {
        Avtp_Ntscf_t* ntscf = (Avtp_Ntscf_t*)common_header;
        stream_id = Avtp_Ntscf_GetStreamId(ntscf);
        acf_list_size = Avtp_Ntscf_GetNtscfDataLength(ntscf);
        first_acf_msg = (Avtp_AcfCommon_t*)&ntscf->payload;
    } else if (subtype == AVTP_SUBTYPE_TSCF) {
        Avtp_Tscf_t* tscf = (Avtp_Tscf_t*)common_header;
        stream_id = Avtp_Tscf_GetStreamId(tscf);
        acf_list_size = Avtp_Tscf_GetStreamDataLength(tscf);
        first_acf_msg = (Avtp_AcfCommon_t*)&tscf->payload;
    }

    // Apply filter
    if (stream_id == gbb_listener->request_stream_id) {
        filter_frame = 0;
    }

    Avtp_AcfCommon_t* next_acf_msg = first_acf_msg;
    if (!filter_frame) {
        size_t next_acf_msg_size = Avtp_AcfCommon_GetAcfMsgLength(next_acf_msg) * 4;
        while (next_acf_msg != NULL) {
            gbb_listener_handle_acf_msg(gbb_listener, next_acf_msg);
            if ((uint8_t*)next_acf_msg + next_acf_msg_size < (uint8_t*)first_acf_msg + acf_list_size) {
                next_acf_msg = (Avtp_AcfCommon_t*)((uint8_t*)next_acf_msg + next_acf_msg_size);
                next_acf_msg_size = Avtp_AcfCommon_GetAcfMsgLength(next_acf_msg);
            } else {
                next_acf_msg = NULL;
                next_acf_msg_size = 0;
            }
        }
    }
}

void gbb_listener_handle_acf_msg(gbb_listener_t* gbb_listener, Avtp_AcfCommon_t* request)
{
    // Prepare TX buffer
    Avtp_Ntscf_t* ntscf = (Avtp_Ntscf_t*)gbb_listener->tx_buf;
    Avtp_Ntscf_Init(ntscf);
    Avtp_Ntscf_SetStreamId(ntscf, gbb_listener->response_stream_id);
    Avtp_Ntscf_SetSv(ntscf, 1);
    Avtp_Gbb_t* gbb = (Avtp_Gbb_t*)&ntscf->payload;
    Avtp_Gbb_Init(gbb);
    Avtp_Gbb_SetByteBusId(gbb, gbb_listener->byte_bus_id);
    Avtp_Gbb_SetOp(gbb, 1);

    uint8_t send_response_flag = 0;
    uint16_t response_size = 0;
    uint8_t acf_msg_type = Avtp_AcfCommon_GetAcfMsgType(request);
    if (acf_msg_type == AVTP_ACF_TYPE_BYTE_BUS) {
        // TODO handle request
        send_response_flag = 1;
    } else if (acf_msg_type == AVTP_ACF_TYPE_BYTE_BUS_BRIEF) {
        // TODO handle request
        send_response_flag = 1;
    }

    // Copy response data
    memcpy(&gbb->payload, (uint8_t[]){0x1, 0x2, 0x3, 0x4, 0x5, 0x6}, 6); // just dummy data for now
    response_size = 6;

    // Set GBB response length and padding
    uint8_t pad = 0;
    if (response_size % 4 != 0) {
        pad = 4 - response_size % 4;
    }
    uint8_t quadlets = (AVTP_GBB_HEADER_LEN + response_size + pad) / 4;
    Avtp_Gbb_SetAcfMsgLength(gbb, quadlets);
    Avtp_Gbb_SetPad(gbb, pad);
    Avtp_Ntscf_SetNtscfDataLength(ntscf, quadlets * 4);
    gbb_listener->tx_buf_level = AVTP_NTSCF_HEADER_LEN + quadlets * 4;

    if (send_response_flag) {
        int res = sendto(
            gbb_listener->socket_fd,
            gbb_listener->tx_buf,
            gbb_listener->tx_buf_level,
            0,
            (struct sockaddr *)&gbb_listener->dst_socket_addr,
            sizeof(gbb_listener->dst_socket_addr));
        if (res < 0) {
            fprintf(stderr, "Failed to transmit response\n");
        }
    }
}

int main(int argc, char** argv)
{
    // Set default config parameters
    gbb_listener_cfg_t cfg = {0};
    cfg.priority = 0;
    cfg.request_stream_id = REQUEST_STREAM_ID;
    cfg.response_stream_id = RESPONSE_STREAM_ID;
    cfg.byte_bus_id = BYTE_BUS_ID;

    // Parse command line arguments
    struct argp argp = { argp_options, parse_args };
    argp_parse(&argp, argc, argv, 0, NULL, &cfg);

    // Initialize talker app
    gbb_listener_t gbb_listener;
    gbb_listener_init(&gbb_listener, &cfg);

    // Wait for response
    struct pollfd fds;
    fds.fd = gbb_listener.socket_fd;
    fds.events = POLLIN;
    while (1) {
        if (poll(&fds, 1, -1)) {
            if (fds.revents & POLLIN) {
                size_t max_len = RX_BUFFER_SIZE;
                ssize_t pdu_len = read(fds.fd, gbb_listener.rx_buf, max_len);
                if (pdu_len > AVTP_COMMON_HEADER_LEN) {
                    gbb_listener.rx_buf_level = pdu_len;
                    gbb_listener_handle_request(&gbb_listener);
                } else {
                    gbb_listener.rx_buf_level = 0;
                }
            }
        }
    }

    return EXIT_SUCCESS;
}
