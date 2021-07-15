#include "stm32_gpio.h"

#include <arch/board/buttons_ioctl.h>
#include <nuttx/fs/fs.h>
#include <stdio.h>

#define BUTTONS_NUM	3

typedef struct file file_t;

static int buttons_open(file_t *filep);
static int buttons_close(file_t *filep);
static ssize_t buttons_read(file_t *filep, char *buf, size_t buflen);
static ssize_t buttons_write(file_t *filep, const char *buf, size_t buflen);
static int buttons_ioctl(file_t *filep, int cmd, unsigned long arg);

static const int GPIO_BUTTON[BUTTONS_NUM] = {
	(GPIO_INPUT | GPIO_CNF_INPULLUP | GPIO_EXTI | GPIO_PORTA | GPIO_PIN0),
	(GPIO_INPUT | GPIO_CNF_INPULLUP | GPIO_EXTI | GPIO_PORTA | GPIO_PIN1),
	(GPIO_INPUT | GPIO_CNF_INPULLUP | GPIO_EXTI | GPIO_PORTA | GPIO_PIN2),
};

static const struct file_operations buttons_ops = {
	buttons_open,
	buttons_close,
	buttons_read,
	buttons_write,
	0,
	buttons_ioctl,
};

static int buttons_open(file_t *filep)
{
	return OK;
}

static int buttons_close(file_t *filep)
{
	return OK;
}

static ssize_t buttons_read(file_t *filep, char *buf, size_t buflen)
{
	for (size_t i = 0; i < BUTTONS_NUM; ++i)
		buf[i] = (stm32_gpioread(GPIO_BUTTON[i]) + '0');

	buf[BUTTONS_NUM] = '\n';

	return BUTTONS_NUM + 1;
}

static ssize_t buttons_write(file_t *filep, const char *buf, size_t buflen)
{
	return OK;
}

static int buttons_ioctl(file_t *filep, int cmd, unsigned long arg)
{
	int ret = OK;
	buttons_irq_req *request = (buttons_irq_req *)arg;
	switch (cmd) {
		case BUTTON_IRQ_ENABLE:
			ret = stm32_gpiosetevent(GPIO_BUTTON[request->num],
					request->trigger & BUTTON_TRIGGER_RISING,
					request->trigger & BUTTON_TRIGGER_FALLING,
					true,
					request->callback,
					request->cbargs);
			break;
		case BUTTON_IRQ_DISABLE:
			ret = stm32_gpiosetevent(GPIO_BUTTON[request->num],
					false,
					false,
					false,
					NULL,
					NULL);	
			break;
		case BUTTON_GET_STATE:
			ret = stm32_gpioread(GPIO_BUTTON[request->num]);
			break;
		default:
			return -1;
	};
	return ret;
}

int up_buttons(void)
{
	int ret;

	for (size_t i = 0; i < BUTTONS_NUM; ++i)
		stm32_configgpio(GPIO_BUTTON[i]);

	ret = register_driver("/dev/buttons", &buttons_ops, 0444, NULL);

	return ret;
}
