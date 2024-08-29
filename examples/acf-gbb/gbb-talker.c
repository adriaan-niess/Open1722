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
#include <argp.h>
#include <unistd.h>
#include <linux/if.h>
#include <linux/if_ether.h>
#include <linux/if_packet.h>

#include "common/common.h"
#include "avtp/CommonHeader.h"
#include "avtp/acf/Ntscf.h"
#include "avtp/acf/Tscf.h"
#include "avtp/acf/Gbb.h"

/**
 * Timeout when waiting for responses.
 */
#define REQUEST_TIMEOUT_MS 1000

/**
 * Stream ID to used for SPI transmit requests.
 */
#define REQUEST_STREAM_ID 0xAABBCCDDEEFF0001

/**
 * Max size of SPI transmit. Should fit into a single Ethernet frame because
 * segmentation is not supported.
 */
#define MAX_SPI_TRANSMIT_SIZE 1000

/**
 * Max number of in-flight requests
 */
#define MAX_PARALLEL_TRANSACTIONS ((1 << 8) - 1)

#define BYTE_BUS_ID 55

/**
 * AVTP headers for transmitting GBB requests/responses.
 */
typedef struct {
    Avtp_Ntscf_t ntscf;
    Avtp_Gbb_t gbb;
} spi_avtp_pdu_t;

typedef struct {
    uint8_t num;
    uint8_t in_use;
    uint32_t timeout_s;
    uint32_t timeout_ns;
} transaction_number_t;

/**
 * Configuration parameters for SPI talker app.
 */
typedef struct {
    char ifname[IFNAMSIZ];
    int priority;
    uint64_t max_transit_time_ns;
    uint64_t request_stream_id;
    uint8_t dst_macaddr[ETH_ALEN];
    int response_timeout_ms;
    uint16_t byte_bus_id;
} spi_talker_cfg_t;

/**
 * State of SPI talker app.
 */
typedef struct {
    char ifname[IFNAMSIZ];
    int priority;
    uint64_t max_transit_time_ns;
    uint64_t request_stream_id;
    uint8_t dst_macaddr[ETH_ALEN];

    uint16_t byte_bus_id;

    /** Max time to wait for response */
    int response_timeout_ms;

    /** Sequence counter. */
    uint8_t seq_num;

    /** Socket file descriptor for sending packets. */
    int socket_fd;

    /** Socket address for sending packets. */
    struct sockaddr_ll dst_socket_addr;

    /** File descriptor from whom the input data is red. */
    int input_fd;

    /** Buffer to construct Ethernet frames for transmission */
    uint8_t tx_buf[sizeof(spi_avtp_pdu_t) + MAX_SPI_TRANSMIT_SIZE];

    /** Buffer for receiving Ethernet frames. */
    uint8_t rx_buf[sizeof(spi_avtp_pdu_t) + MAX_SPI_TRANSMIT_SIZE];
} spi_talker_t;

static struct argp_option argp_options[] = {
    {"dst-addr", 'd', "MACADDR", 0, "Stream Destination MAC address" },
    {"ifname", 'i', "IFNAME", 0, "Network Interface" },
    {"max-transit-time", 'm', "MSEC", 0, "Maximum Transit Time in ms" },
    {"prio", 'p', "NUM", 0, "SO_PRIORITY to be set in socket" },
    { 0 }
};

error_t parse_args(int key, char *arg, struct argp_state *argp_state);
void init_spi_talker(spi_talker_t* spi_talker, spi_talker_cfg_t* spi_talker_cfg);
int remote_spi_transmit(spi_talker_t* spi_talker, uint8_t* write, uint8_t* read, uint16_t write_size, uint16_t read_size);
int remote_spi_transmit_async(spi_talker_t* spi_talker, uint8_t* write, uint8_t* read, uint16_t write_size, uint16_t read_size);

error_t parse_args(int key, char *arg, struct argp_state *argp_state)
{
    spi_talker_cfg_t* cfg = argp_state->input;

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
    }
    return 0;
}

void init_spi_talker(spi_talker_t* spi_talker, spi_talker_cfg_t* spi_talker_cfg)
{
    memset(spi_talker, 0, sizeof(spi_talker_t));

    // Copy configuration
    spi_talker->priority = spi_talker_cfg->priority;
    spi_talker->max_transit_time_ns = spi_talker_cfg->max_transit_time_ns;
    spi_talker->response_timeout_ms = spi_talker_cfg->response_timeout_ms;
    spi_talker->request_stream_id = spi_talker_cfg->request_stream_id;
    spi_talker->byte_bus_id = spi_talker_cfg->byte_bus_id;
    memcpy(spi_talker->dst_macaddr, spi_talker_cfg->dst_macaddr, ETH_ALEN);
    memcpy(spi_talker->ifname, spi_talker_cfg->ifname, IFNAMSIZ);

    spi_talker->seq_num = 0;

    // Check if interface name is not empty
    if (strcmp(spi_talker->ifname, "") == 0) {
        fprintf(stderr, "No ifname argument was provided (-i, --ifname)\n");
        exit(EXIT_FAILURE);
    }

    // Open socket
    spi_talker->socket_fd = create_talker_socket(spi_talker->priority);
    if (spi_talker->socket_fd < 0) {
        fprintf(stderr, "Failed to open socket!\n");
        exit(EXIT_FAILURE);
    }

    // Set socket address
    int res = setup_socket_address(
            spi_talker->socket_fd,
            spi_talker->ifname,
            spi_talker->dst_macaddr,
            ETH_P_TSN,
            &spi_talker->dst_socket_addr);
    if (res < 0) {
        fprintf(stderr, "Failed to set up socket address!\n");
        exit(EXIT_FAILURE);
    }

    // Prepare TX buffer
    spi_avtp_pdu_t* pdu = (spi_avtp_pdu_t*)spi_talker->tx_buf;
    Avtp_Ntscf_Init(&pdu->ntscf);
    Avtp_Ntscf_SetStreamId(&pdu->ntscf, spi_talker->request_stream_id);
    Avtp_Ntscf_SetSv(&pdu->ntscf, 1);
    Avtp_Gbb_Init(&pdu->gbb);
    Avtp_Gbb_SetByteBusId(&pdu->gbb, spi_talker->byte_bus_id);
    Avtp_Gbb_SetOp(&pdu->gbb, 0);
}

int remote_spi_transmit(spi_talker_t* spi_talker, uint8_t* write, uint8_t* read, uint16_t write_size, uint16_t read_size)
{
    // if (spi_talker->num_free_transaction_ids <= 0) {
    //     fprintf(stderr, "Run out of transaction numbers!\n");
    //     return -EINVAL;
    // }

    if (write_size > MAX_SPI_TRANSMIT_SIZE) {
        fprintf(stderr, "Max SPI transmit size exceeded!\n");
        return -EINVAL;
    }

    uint8_t transaction_num = 42; // TODO obtain valid transaction number
    
    spi_avtp_pdu_t* pdu = (spi_avtp_pdu_t*)spi_talker->tx_buf;
    Avtp_Ntscf_SetSequenceNum(&pdu->ntscf, spi_talker->seq_num++);
    Avtp_Ntscf_SetNtscfDataLength(&pdu->ntscf, AVTP_GBB_HEADER_LEN + write_size);
    uint8_t padding = 0;
    Avtp_Gbb_SetAcfMsgLength(&pdu->gbb, (AVTP_GBB_HEADER_LEN + write_size) / 4); // TODO consider padding
    Avtp_Gbb_SetReadSize(&pdu->gbb, read_size);
    memcpy(pdu->gbb.payload, write, write_size);

    // Send to network
    size_t pdu_len = AVTP_NTSCF_HEADER_LEN + AVTP_GBB_HEADER_LEN + write_size + padding;
    int res = sendto(
            spi_talker->socket_fd,
            pdu,
            pdu_len,
            0,
            (struct sockaddr *)&spi_talker->dst_socket_addr,
            sizeof(spi_talker->dst_socket_addr));
    if (res < 0) {
        fprintf(stderr, "Failed to transmit\n");
    }

    return 0;
}

int remote_spi_transmit_async(spi_talker_t* spi_talker, uint8_t* write, uint8_t* read, uint16_t write_size, uint16_t read_size)
{
    // TODO
}

int main(int argc, char** argv)
{
    // Parse command line arguments
    spi_talker_cfg_t cfg = {0};
    struct argp argp = { argp_options, parse_args };
    argp_parse(&argp, argc, argv, 0, NULL, &cfg);

    // Set static parameters
    cfg.response_timeout_ms = REQUEST_TIMEOUT_MS;
    cfg.request_stream_id = REQUEST_STREAM_ID;
    cfg.byte_bus_id = BYTE_BUS_ID;

    // Initialize talker app
    spi_talker_t spi_talker;
    init_spi_talker(&spi_talker, &cfg);

    // Send data
    int count = 0;
    uint8_t write[4] = {0x1, 0x2, 0x3, 0x4};
    uint8_t read[4];
    while (1) {
        // printf("Count %d\n", count++);
        remote_spi_transmit(&spi_talker, write, read, 4, 4);
        usleep(1000 * 1000);
    }

    return EXIT_SUCCESS;
}
