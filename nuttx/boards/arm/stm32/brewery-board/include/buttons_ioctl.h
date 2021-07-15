#ifndef BUTTONS_IOCTL_H_
#define BUTTONS_IOCTL_H_

#include <stdint.h>

typedef enum button_ioctl_cmd_t {
	BUTTON_IRQ_ENABLE, BUTTON_IRQ_DISABLE, BUTTON_GET_STATE,
} button_ioctl_cmd;

typedef enum button_trigger_t {
	BUTTON_TRIGGER_RISING = 0b10, BUTTON_TRIGGER_FALLING = 0b1,
} button_trigger;

typedef enum button_num_t {
	BUTTON_0, BUTTON_1, BUTTON_2,
} button_num;

typedef struct callback_args_t {
	char msg[256];
} callback_args;

typedef struct buttons_irq_req_t {
	button_num num;
	button_trigger trigger;
	callback_args *cbargs;
	int (*callback)(int irq, void *context, void *arg);	
} buttons_irq_req;

#endif	// BUTTONS_IOCTL_H_
