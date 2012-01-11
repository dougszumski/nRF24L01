/*  
 * Based on documentation/spi/spidev_test.c
 * Doug S. Szumski <d.s.szumski@gmail.com> 11-01-2012 and
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

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

static void pabort(const char *s)
{
	perror(s);
	abort();
}

static int fd;
static uint8_t mode;  //require mode 0 for nrf
static uint8_t bits = 8;
static uint32_t speed = 1000;
static uint16_t delay = 10000;
static uint8_t CE_PIN = 203;

void SPI_Init(char *device)
{
	int ret = 0;
	int value = 0;

	printf("Initialising SPI...\n\n");	

	//Open the spidev node
	fd = open(device, O_RDWR);
	if (fd < 0)
		pabort("Can't open SPI device");

	//Setup the GPIOs
	ret = gpio_export(CE_PIN);  //S3C2410_GPG11_EINT19
	ret = gpio_dir_out(CE_PIN);
   
	//SPI modes -- only mode 0 works 
	ret = ioctl(fd, SPI_IOC_WR_MODE, &mode);
	if (ret == -1)
		pabort("can't set spi mode");
	ret = ioctl(fd, SPI_IOC_RD_MODE, &mode);
	if (ret == -1)
		pabort("can't get spi mode");
	printf("SPI mode: %d\n", mode);

	//Bits per word
	ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
	if (ret == -1)
		pabort("can't set bits per word");
	ret = ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits);
	if (ret == -1)
		pabort("can't get bits per word");
	printf("Bits per word: %d\n", bits);

	//Max speed Hz
	ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
	if (ret == -1)
		pabort("can't set max speed hz");
	ret = ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
	if (ret == -1)
		pabort("can't get max speed hz");
	printf("Max speed: %d Hz (%d KHz)\n", speed, speed/1000);

	printf("SPI initialised...\n\n");

	//FIXME: close(fd);
}

void SPI_ReadWrite_Block(uint8_t* tx, uint8_t* rx, int length)
{
	int ret;

	/*
	printf("TX buffer");
	for (ret = 0; ret < length; ret++) {
		printf("%.2X ", tx[ret]);
	}
	puts("");	*/

	struct spi_ioc_transfer tr = {
		.tx_buf = (unsigned long)tx,
		.rx_buf = (unsigned long)rx,
		.len = length,
		.delay_usecs = delay,
		.speed_hz = speed,
		.bits_per_word = bits,
	};

	ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
	if (ret == 0)
		pabort("can't send spi message");

	/*
	printf("RX buffer");
	for (ret = 0; ret < length; ret++) {
		printf("%.2X ", rx[ret]);
	}
	puts("");*/
}

uint8_t SPI_Write_Byte(uint8_t tx_val)
{
	int ret;
	uint8_t tx [] = {tx_val};
	uint8_t rx [] = {0x00};

	struct spi_ioc_transfer tr = {
		.tx_buf = (unsigned long)tx,
		.rx_buf = (unsigned long)rx,
		.len = ARRAY_SIZE(tx),
		.delay_usecs = delay,
		.speed_hz = speed,
		.bits_per_word = bits,
	};

	ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
	if (ret == 0)
		pabort("can't send spi message");
	
	return *rx;

}

void CE_HIGH()
{
	int ret;
	struct timespec ts;
	ts.tv_sec = 0;
        ts.tv_nsec = 20000000;
    //    nanosleep (&ts, NULL);
	ret = gpio_value(203, 1);
	if (ret == -1)
		pabort("Can't set CE high");
    //nanosleep (&ts, NULL);
}

void CE_LOW()
{
	int ret;
	struct timespec ts;
	ts.tv_sec = 0;
        ts.tv_nsec = 20000000;
    //    nanosleep (&ts, NULL);
	ret = gpio_value(203, 0);
	if (ret == -1)
		pabort("Can't set CE low");
    //nanosleep (&ts, NULL);
	
}
