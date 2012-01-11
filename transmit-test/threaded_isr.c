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

#include "threaded_isr.h"

 static void interrupt_handler(unsigned);

/*
int main(void)
{
	int ret;

	char device [] = "/dev/uio0";
	
	//make sure this doesnt go out of scope before thread finsihes
	struct isr_parameters para;
	para.device_node = device;
	para.handler = interrupt_handler;

	//isr_blocking_read(device,interrupt_handler);
	
	ret = pthread_create( &interrupt_thread, NULL, isr_blocking_read, (void *) &para);
	//isr_blocking_read(( (void *) &para));
	//pthread_join(interrupt_thread, NULL);
	

	while(1) {

		printf("hello");
		sleep(1);


	}
	
*/
	/*
	uiofd = open("/dev/uio0", O_RDONLY);
	if (uiofd < 0) {
		perror("unable to open uio node\n");
		return errno;
	}



		fd_set set;
       struct timeval timeout;
		
		while(1) {

	   FD_ZERO (&set);
       FD_SET (uiofd, &set);



	   errno = select (FD_SETSIZE,
                                          &set, NULL, NULL,
              
											NULL		);
		
		err = read(uiofd, &icount, 4);
		if (err != 4) {
			perror("error reading uio node\n");
			return errno;
	
	  }
		
	printf("icount: %d",icount);
	printf("caught me an interrupt\n");
	}

	return errno;

*/

//	return 0;
//}



static void interrupt_handler(unsigned interrupts) {
	//printf("in interrupt handler");
	printf("%d\t",interrupts);
	fflush(stdout);
}


pthread_mutex_t * isr_get_mutex(void)
{
return &isr_mutex;

}

int uio_select(fd_set *set) {
	int err = 0;
	//short success = 0;
	//fix me: pass this in	
	struct timeval timeout;
	//timeout.tv_sec = 10; 
   	timeout.tv_usec = 1;

	//
		// returns 0 on thread change ? - retry
		//while (!success) {
			
		err = select (FD_SETSIZE,
                                          set, NULL, NULL,
											&timeout		);
		/* now using a timeout
		switch (err) {
		case 1:
			success = 1;
			break;
		default:
			break;
		}
		
		
}*/
		//printf("Err: %d\n",err);
		return err;
}

int uio_read(int *fd,  unsigned * icount) {	 
	//printf("in uio_read");
	int status = 0;	
	int err = 0;
	//int uiofd = open("/dev/uio0", O_RDONLY);
	//printf("fd: %d\n",*fd);
	//printf("icount: %d\n",*icount);
	err = read(*fd, icount, 4);
		if (err != 4) {
			perror("error reading uio");
			return -1;
		}
	return status;
}

//int isr_blocking_read(char * node, void (*fp)(unsigned) ) {
void * isr_blocking_read(void * para_in ) {
	
	//printf("started isr thread\n");
	struct isr_parameters para = *(struct isr_parameters *) para_in;	
	char * node = para.device_node;
	void (*fp)(unsigned) = para.handler;
	int uiofd;
	unsigned interrupt_count;
	int err;
	int status = 0;
	fd_set set;
	FD_ZERO (&set);

	
	uiofd = open(node, O_RDONLY);
	if (uiofd < 0) {
		perror("unable to open uio node\n");
		return (void *)errno;
	}

	FD_SET (uiofd, &set);
	

	while (status >= 0) {
	//while(0){		
		//printf("isr loop\n");
		pthread_mutex_lock( &isr_mutex );
		err = uio_select(&set);	
		switch (err) {
		case 1:
			 if (uio_read(&uiofd,&interrupt_count) < 0 ) {
					status = -1;
					break;
				}
			//uio_read(&uiofd,&interrupt_count);
			//read(uiofd, &interrupt_count, 4);
			fp(interrupt_count);
			//pthread_yield();
			//fp(0);
			break;
		case -1:
			status = -2;
			break;
		default:
			break;
		}
		//break;
		//sleep(1);
		pthread_mutex_unlock( &isr_mutex );
		pthread_yield();
	}
	
	
	close(uiofd);
	return (void *) status;
	
	//return 0;
}
