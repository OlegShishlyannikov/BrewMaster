#include <nuttx/config.h>
#include <fcntl.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <arch/board/buzzer_ioctl.h>

int main(int argc, FAR char *argv[])
{
	int fd = open("/dev/buzzer", O_RDONLY);

	buzzer_beep_req bbr;
	bbr.n = 100;
	bbr.up = 10;
	bbr.down = 10;
	bbr.delay = 100;

	int ret = ioctl(fd, BUZZER_BEEP, &bbr);

	return 0;
}
