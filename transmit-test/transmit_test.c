/*  
 * Doug S. Szumski <d.s.szumski@gmail.com> and
 * Will J. Szumski <averylongnamewhichnobodyhas@gmail.com> 11-01-2012
 * 
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
*/

#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include "gpio.h"
#include "spi.h"
#include "radio.h"
#include "packet.h"
#include "threaded_isr.h"

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

volatile uint8_t rxflag = 0;
uint8_t station_addr[5] = { 0xE4, 0xE4, 0xE4, 0xE4, 0xE4 };
uint8_t my_addr[5] = { 0x91, 0x76, 0x54, 0x32, 0x10 };
radiopacket_t packet;

char device_uio [] = "/dev/uio0";
	struct isr_parameters para;

void setup()
{
	printf("size of radiopacket: %d \n",sizeof(radiopacket_t));
	printf("size of payload: %d \n", sizeof(payloadformat_t)); 
	printf("size of pf_message: %d \n", sizeof(pf_message_t)); 
	printf("size of packet_type: %d \n", sizeof(PACKET_TYPE));
	printf("u_int16: %d: %d\n",sizeof(uint16_t));

	SPI_Init(device);
	printf(" Spi Init complete \n");
	Radio_Init();
	printf(" Radio Init complete \n");
	// configure the receive settings for radio pipe 0
	Radio_Configure_Rx(RADIO_PIPE_0, my_addr, ENABLE);
	printf(" radio configure (receive) complete \n");
	// configure radio transciever settings.
	Radio_Configure(RADIO_2MBPS, RADIO_HIGHEST_POWER);
	printf(" radio configure (transmit) complete \n");
	// interrupt monitoring loop
	
	int ret;
	//make sure this doesnt go out of scope before thread finsihes

	//isr_blocking_read(device,interrupt_handler);
	para.device_node = device_uio;
	para.handler = interrupt_handler;
	ret = pthread_create( &interrupt_thread, NULL, isr_blocking_read, (void *) &para);
	
	printf(" Setup complete \n");
}

void transmit() {
	// load up the packet contents	
	packet.type = MESSAGE;
	packet.timestamp = 0xbeef;
	memcpy(packet.payload.message.address, my_addr, RADIO_ADDRESS_LENGTH);
	packet.payload.message.messageid = 55;
	snprintf((char*)packet.payload.message.messagecontent, sizeof(packet.payload.message.messagecontent), 		"Test message.");
	Radio_Set_Tx_Addr(station_addr);
	Radio_Transmit(&packet, RADIO_WAIT_FOR_TX);
}
 
void loop()
{
	printf("pre transmit\n");	
	transmit();
	printf("transmitted \n");
	if (rxflag)
	{

		if (Radio_Receive(&packet) != RADIO_RX_MORE_PACKETS)
		{
			// if there are no more packets on the radio, clear the receive flag;
			// otherwise, we want to handle the next packet on the next loop iteration.
			rxflag = 0;
		}
		if (packet.type == ACK)
		{
			//exit(0);
			//Have some tea
		} else {
			//printf("Packet type %X, ACK: %X, MESSAGE: %X \n", packet.type, ACK, MESSAGE);
		}

		printf("RX flag\n");
		printf("Packet type %X, ACK: %X, MESSAGE: %X \n", packet.type, ACK, MESSAGE);	
		printf("Msg: %s\n", packet.payload.message.messagecontent);
		printf("timestamp: %x\n",packet.timestamp);
	}
	struct timespec ts;
	ts.tv_sec = 0;
        ts.tv_nsec = 200000000;
     nanosleep (&ts, NULL);
}
 
void radio_rxhandler(uint8_t pipe_number)
{
	rxflag = 1;
}

// Arduino's default main function (included here for clarity)
int main()
{
	setup();
	for (;;) loop();
}
