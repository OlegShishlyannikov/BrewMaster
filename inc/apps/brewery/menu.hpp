#ifndef MENU_HPP
#define MENU_HPP

#include "sys/system.hpp"
#include "util/hash/sha256.hpp"
#include "json/json.h"

struct menu_entry_s {
  struct jfes_value *json_entry;
  void (*draw)(const sys_impl_s *, const jfes_value *);
  void (*on_trigger)(const sys_impl_s *, const jfes_value *);
  struct menu_entry_s *children;
  size_t children_n;
};

struct menu_s {
  explicit menu_s(struct jfes_value *);
  virtual ~menu_s() = default;

private:
  struct menu_entry_s *initialize_(struct menu_entry_s *, struct jfes_value *);
  struct menu_entry_s *root_;
};

#endif /* MENU_HPP */
