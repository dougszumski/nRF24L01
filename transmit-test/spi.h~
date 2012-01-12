/*  
 * Based on documentation/spi/spidev_test.c
 * Doug S. Szumski <d.s.szumski@gmail.com> 11-01-2012 and
 * Will J. Szumski <averylongnamewhichnobodyhas@gmail.com> 
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

#ifndef SPI_H_
#define SPI_H_

#include <stdint.h>
#include <unistd.h>
#include <stdio.h>

//SPI_DEV node
static char *device = "/dev/spidev0.0";

//Init SPI, returns file descriptor
void SPI_Init(char *device);

void SPI_ReadWrite_Block (uint8_t* tx, uint8_t* rx, int length);

uint8_t SPI_Write_Byte (uint8_t tx);

void CE_HIGH();

void CE_LOW();


#endif 
