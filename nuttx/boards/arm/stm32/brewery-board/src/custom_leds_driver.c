#include "stm32_gpio.h"

#include <nuttx/fs/fs.h>

#include <ctype.h>
#include <stdio.h>

#define LEDS_NUM	3
#define LED_ON		1
#define LED_OFF		0

typedef struct file file_t;

static int leds_open(file_t *filep);
static int leds_close(file_t *filep);
static ssize_t leds_read(file_t *filep, char *buf, size_t buflen);
static ssize_t leds_write(file_t *filep, const char *buf, size_t buflen);

static const int GPIO_LED[LEDS_NUM] = {
	(GPIO_OUTPUT | GPIO_MODE_2MHz | GPIO_PORTC | GPIO_PIN12),
	(GPIO_OUTPUT | GPIO_MODE_2MHz | GPIO_PORTC | GPIO_PIN13),
	(GPIO_OUTPUT | GPIO_MODE_2MHz | GPIO_PORTC | GPIO_PIN14),
};


static const struct file_operations leds_ops = {
	leds_open, 
	leds_close,
	leds_read,
	leds_write,
	0,
	0,
};

static int leds_open(file_t *filep)
{
	return OK;
}

static int leds_close(file_t *filep)
{
	return OK;
}

static ssize_t leds_read(file_t *filep, char *buf, size_t buflen)
{
	if(!buf || buflen < LEDS_NUM + 1)
		return -EINVAL;

	for (int i = 0; i < LEDS_NUM; ++i)
		buf[i] = stm32_gpioread(GPIO_LED[i]) + '0';

	buf[LEDS_NUM] = '\n';

	return LEDS_NUM + 1;
}

static ssize_t leds_write(file_t *filep, const char *buf, size_t buflen)
{
	if (!buf || buflen != LEDS_NUM + 1)	// due to '\n' symbol
		return -EINVAL;
	
	for (int i = 0; i < LEDS_NUM; ++i) {
		if (!isdigit(buf[i]))
			return -EINVAL;

		int led_v = buf[i] - '0';
		if (led_v == LED_ON || led_v == LED_OFF)
			stm32_gpiowrite(GPIO_LED[i], led_v);
	}

	return buflen;
}

int up_leds(void)
{
	int ret;

	for (int i = 0; i < LEDS_NUM; ++i)
		stm32_configgpio(GPIO_LED[i]);

	ret = register_driver("/dev/leds", &leds_ops, 0444, NULL);

	return ret;
}
