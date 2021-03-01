#include "tasks/events_worker.hpp"

xQueueHandle events_worker_queue;

void events_worker_task_code(void *params) {
  BaseType_t os_rc;
  int32_t rc;
  const struct events_worker_s *ew = reinterpret_cast<const struct events_worker_s *>(params);
  typename events_worker_s::event_s buffer;

  if (!events_worker_queue) {
    goto error_state;
  }

  while (true) {
    if ((os_rc = xQueueReceive(events_worker_queue, &buffer, portMAX_DELAY)) != pdPASS) {
      continue;
    }

    ew->handle_event(&buffer);
  }

error_state:
  vTaskDelete(nullptr);
  while (true)
    ;
}
