#include <nuttx/config.h>
#include <fcntl.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <arch/board/buttons_ioctl.h>

int func1(int irq, void *context, void *arg)
{
	callback_args *cbargs = (callback_args *)arg;
	printf("%s", cbargs->msg);
}

int main(int argc, FAR char *argv[])
{
	int fd = open("/dev/buttons", O_RDONLY);

	buttons_irq_req bir;
	bir.num = BUTTON_0;
	bir.trigger = BUTTON_TRIGGER_RISING;

	callback_args cbargs0;
	strcpy(cbargs0.msg, "Hello from BUTTON_0!\n");

	bir.cbargs = &cbargs0;
	bir.callback = func1;

	int ret = ioctl(fd, BUTTON_IRQ_ENABLE, &bir);
	printf("STATUS0 = %d\n", ret);

	bir.num = BUTTON_1;

	callback_args cbargs1;
	strcpy(cbargs1.msg, "Hello from BUTTON_1!\n");

	bir.cbargs = &cbargs1;

	ret = ioctl(fd, BUTTON_IRQ_ENABLE, &bir);
	printf("STATUS1 = %d\n", ret);

	bir.num = BUTTON_2;
	callback_args cbargs2;
	strcpy(cbargs2.msg, "Hello from BUTTON_2!\n");
	bir.cbargs = &cbargs2;

	ret = ioctl(fd, BUTTON_IRQ_ENABLE, &bir);
	printf("STATUS2 = %d\n", ret);

	return 0;
}
