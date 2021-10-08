#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"

#include "sys/events_worker.hpp"

extern xQueueHandle events_worker_queue;

void events_worker_s::init() const {
  events_worker_queue = xQueueCreate(128u, sizeof(struct events_worker_s::event_s));
}

void events_worker_s::handle_event(const struct events_worker_s::event_s *event) const {
  if (event->handler != nullptr) {
    event->handler(event->data, event->size);
  }

  if (event->data) {
	std::free(event->data);
  }
}
