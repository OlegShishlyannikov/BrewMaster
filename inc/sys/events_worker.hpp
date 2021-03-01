#ifndef EVENTS_WORKER_HPP
#define EVENTS_WORKER_HPP

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <memory>
#include <sys/types.h>
#include <type_traits>
#include <util/hash/fnv1a.hpp>

struct events_worker_s {
  static constexpr uint32_t max_events = 256u;

  struct event_s {
    void (*handler)(const void *, size_t);
    void *data;
    size_t size;
  };

  void init() const;
  void handle_event(const struct event_s *) const;
};

/* Driver object singletone generator implementation (no dependencies) */
static auto &make_ew() {
  static const std::unique_ptr<struct events_worker_s> inst{new events_worker_s};
  return *inst;
}

#endif /* EVENTS_WORKER_HPP */
