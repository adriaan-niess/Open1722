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

#define BYTE_BUS_ID 55

#define MAX_AVTP_HEADER_SIZE (AVTP_COMMON_HEADER_LEN + AVTP_TSCF_HEADER_LEN)

#define MAX_GBB_TRANSMIT_SIZE 1000

#define RX_BUFFER_SIZE (MAX_AVTP_HEADER_SIZE + MAX_GBB_TRANSMIT_SIZE)

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
    uint8_t txBuffer[TX_BUFFER_SIZE];

    /** Buffer for request PDUs. */
    uint8_t rxBuffer[RX_BUFFER_SIZE];
} gbb_listener_t;

static struct argp_option argp_options[] = {
    {"dst-addr", 'd', "MACADDR", 0, "Stream Destination MAC address" },
    {"ifname", 'i', "IFNAME", 0, "Network Interface" },
    {"max-transit-time", 'm', "MSEC", 0, "Maximum Transit Time in ms" },
    {"prio", 'p', "NUM", 0, "SO_PRIORITY to be set in socket" },
    { 0 }
};

error_t parse_args(int key, char *arg, struct argp_state *argp_state);
void init_gbb_listener(gbb_listener_t* gbb_listener, gbb_listener_cfg_t* cfg);

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

void init_gbb_listener(gbb_listener_t* gbb_listener, gbb_listener_cfg_t* cfg)
{
    memset(gbb_listener, 0, sizeof(gbb_listener_t));

    // Copy configuration
    memcpy(gbb_listener->ifname, cfg->ifname, IFNAMSIZ);
    gbb_listener->priority = cfg->priority;
    gbb_listener->request_stream_id = cfg->request_stream_id;
    gbb_listener->response_stream_id = cfg->response_stream_id;
    memcpy(gbb_listener->dst_macaddr, cfg->dst_macaddr, ETH_ALEN);
    gbb_listener->byte_bus_id = cfg->byte_bus_id;

    // Initial other GBB listener state
    gbb_listener->seq_num_request = 0;
    gbb_listener->seq_num_response = 0;

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

    // Prepare TX buffer
    // TODO
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
    init_gbb_listener(&gbb_listener, &cfg);

    return EXIT_SUCCESS;
}
