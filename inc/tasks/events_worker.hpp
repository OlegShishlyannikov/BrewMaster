#ifndef EVENTS_WORKER_TASK_HPP
#define EVENTS_WORKER_TASK_HPP

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"

#include "fio/unistd.hpp"
#include "sys/events_worker.hpp"

extern xQueueHandle events_worker_queue;
void events_worker_task_code(void *params);

#endif /* EVENTS_WORKER_TASK_HPP */
