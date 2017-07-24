/*
 * CC1101_PacketStruct.h
 *
 *  Created on: Jun 9, 2017
 *      Author: Tim
 */

#ifndef CC1101_PACKETSTRUCT_H_
#define CC1101_PACKETSTRUCT_H_

/*  TX PACKET   */
#define TX_PACKET_ADDRESS_INDEX     0
#define TX_PACKET_LEDTOG_INDEX      1
#define TX_PACKET_DEVICEID_INDEX    2
#define TX_PACKET_RSSI_INDEX        3
#define TX_PACKET_LQI_INDEX         4

/*  RX PACKET   */
#define RX_PACKET_ADDRESS_INDEX     0
#define RX_PACKET_LEDTOG_INDEX      1
#define RX_PACKET_POWER_INDEX       2
#define RX_PACKET_TXLENGTH_INDEX    3

/*  RF Power Level  */
#define ZERO_dBm        0x0041
#define TEN_dBm         0x38D3

/* Blanket Octo */
#define SW_CC_LED_RED               0x0008
#define SW_CC_LED_BLUE              0x0004
#define SW_CC_LED_ORANGE            0x0002

#define VSU_ADDRESS     0x01
#endif /* CC1101_PACKETSTRUCT_H_ */
