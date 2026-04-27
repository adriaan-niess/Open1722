Developer Guide
===============

.. code-block:: c

    // my_1722_pdu.h

    #include "avtp/Udp.h"
    #include "avtp/acf/Tscf.h"
    #include "avtp/acf/Can.h"
    #include "avtp/acf/Lin.h"

    #define CAN_PAYLOAD_LEN 2
    #define LIN_PAYLOAD_LEN 3

    typedef struct {
        // IEEE 1722 UDP encapsulation header (optional)
        Avtp_Udp_t udp;
        // IEEE 1722 TSCF header
        Avtp_Tscf_t tscf;
        // IEEE 1722 ACF message #1
        Avtp_Can_t can;
        uint8_t canPayload[CAN_PAYLOAD_LEN];
        // IEEE 1722 ACF message #2
        Avtp_Lin_t lin;
        uint8_t linPayload[LIN_PAYLOAD_LEN];
    } My1722Pdu_t;

.. code-block:: c

    // talker.h

    #include "my_1722_pdu.h"

    int main()
    {
        My1722Pdu_t pdu;

        // Init UDP encapsulation header
        Avtp_Udp_Init(&pdu.udp);

        // Init TSCF header
        Avtp_Tscf_Init(&pdu.tscf);
        Avtp_Tscf_SetVersion(&pdu.tscf, 0);
        Avtp_Tscf_SetSequenceNum(&pdu.tscf, 123);
        Avtp_Tscf_SetStreamId(&pdu.tscf, 0xAABBCCDDEEFF);
        Avtp_Tscf_SetTv(&pdu.tscf, 1);
        Avtp_Tscf_SetAvtpTimestamp(&pdu.tscf, 0x11223344);

        // Init CAN ACF message
        Avtp_Can_Init(&pdu.can);
        Avtp_Can_SetCanBusId(&pdu.can, 4);
        uint8_t canFrame[CAN_PAYLOAD_LEN] = {0x11, 0x22};
        memcpy(pdu.can.payload, canFrame, CAN_PAYLOAD_LEN);

        // Init LIN ACF message
        Avtp_Lin_Init(&pdu.lin);
        uint8_t linFrame[LIN_PAYLOAD_LEN] = {0x11, 0x22, 0x33};
        memcpy(pdu.lin.payload, linFrame, LIN_PAYLOAD_LEN);

        // Send packet to network using socket API ...
        uint8_t* data = &pdu;
        size_t dataLen = sizeof(My1722Pdu_t);
        // int socket = ...
        // sendto(socket, data, dataLen, 0);
    }
