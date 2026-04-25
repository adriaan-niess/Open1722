Open1722
=================================

Open1722 is a fork of AVNU/libavtp which is an open source reference implementation of the Audio Video Transport Protocol (AVTP) specified in IEEE 1722-2016 spec. libavtp primarily focuses on audio video data formats of the IEEE 1722-2016 spec.

IEEE 1722 is also gaining a lot of traction in the automotive community, mainly, to bridge fieldbus technologies over automotive Ethernet. In particular the AVTP Control Formats (ACF) specify serialization for a set of data formats relevant for automotive applications (e.g., CAN, LIN, etc.). Open1722 extends/modifies libavtp to also include these ACF formats.

Open1722 is under BSD License. For more information see LICENSE file.

.. note::
   Open1722 is currently incubating and under active development. The APIs are not fully stable and are subject to changes.

.. toctree::
    :maxdepth: 2

    linux-tutorial
    developer-guide
    api
