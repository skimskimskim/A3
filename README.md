A3 (A-cube)
============================
We propose A3, an autonomous and adaptive slot allocation scheme that adjusts the number of slots per slotframe responding to varying traffic loads. A key component of the proposed algorithm is load estimation that fathoms the traffic loads at the remote corresponding nodes in real-time without any explicit control message exchange. The proposed estimation algorithm is protocol independent in that it can be combined with any autonomous scheduling protocols, such as Orchestra and ALICE.

A3 is an autonomous and adaptive slot allocation scheme that adjusts the number of slots per slotframe responding to varying traffic loads.

A3 source code location: ./core/net/mac/tsch/  and  ./apps/alice/

A3 example code location: ./examples/ipv6/rpl-udp-alice-log/

A3 configuration: ./examples/ipv6/rpl-udp-alice-log/project-conf.h (useful values: A3_MANAGEMENT, A3_UNICAST_MAX_REGION, ORCHESTRA_CONF_UNICAST_PERIOD, PERIOD, ALICE1_ORB2_OSB3)


When using this source code, please cite the following paper:

Seohyang Kim, Hyung-Sin Kim, and Chong-kwon Kim, A3 : Adaptive Autonomous Allocation of TSCH Slots, In the 20th ACM/IEEE International Conference on Information Processing in Sensor Networks (IPSN'21), May 18–21, 2021, Nashville, Tennessee, USA.



The Contiki Operating System
============================

[![Build Status](https://travis-ci.org/contiki-os/contiki.svg?branch=master)](https://travis-ci.org/contiki-os/contiki/branches)

Contiki is an open source operating system that runs on tiny low-power
microcontrollers and makes it possible to develop applications that
make efficient use of the hardware while providing standardized
low-power wireless communication for a range of hardware platforms.

Contiki is used in numerous commercial and non-commercial systems,
such as city sound monitoring, street lights, networked electrical
power meters, industrial monitoring, radiation monitoring,
construction site monitoring, alarm systems, remote house monitoring,
and so on.

For more information, see the Contiki website:

[http://contiki-os.org](http://contiki-os.org)
