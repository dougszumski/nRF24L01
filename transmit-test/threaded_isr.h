/* 
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

#ifndef THREADED_ISR
#define THREADED_ISR


#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <pthread.h>
#include <sys/select.h>


static pthread_t interrupt_thread;

extern void * isr_blocking_read(void * );
extern int uio_select(fd_set *);
extern pthread_mutex_t * isr_get_mutex(void);
extern int uio_read(int *,  unsigned * );



//get lock on mutex before transmission or monitoring interrupt
static pthread_mutex_t isr_mutex = PTHREAD_MUTEX_INITIALIZER;


struct isr_parameters {
	char * device_node;
	void (*handler)(unsigned);
};

#endif
