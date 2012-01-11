#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>

int main()
{
	int uiofd;
	int configfd;
	int err;
	int i;
	unsigned icount;
	unsigned char command_high;

	uiofd = open("/dev/uio0", O_RDONLY);
	if (uiofd < 0) {
		perror("uio open:");
		return errno;
	}

		err = read(uiofd, &icount, 4);
		if (err != 4) {
			perror("uio read:");
		}


	return errno;
}

