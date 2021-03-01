#ifndef ARGPARSE_H
#define ARGPARSE_H

#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum ClArgType {
  cl_arg_last = -1,
  cl_arg_pos,
  cl_arg_flag,
  cl_arg_string,
  cl_arg_int,
  cl_arg_real,
  cl_arg_strlist,
  cl_arg_intlist,
  cl_arg_reallist,
  cl_arg_sequence
} ClArgType_t;

typedef struct ClArg {
  ClArgType_t type;
  const char *name;
  void *value;
  int size;
  int mdty;
} ClArg_t;

void parse_cl_args(int argc, char *argv[], ClArg_t *cl_arg);

void *get_cl_arg(ClArg_t *arg, const char *name, int *size);

#define last_arg                                                                                                       \
  { cl_arg_last, NULL, NULL, 0, 0 }
#define pos_arg(_name)                                                                                                 \
  { cl_arg_pos, _name, NULL, 0, 1 }
#define trailing_arg(_name)                                                                                            \
  { cl_arg_pos, _name, NULL, 0, 0 }
#define flag_arg(_name)                                                                                                \
  { cl_arg_flag, _name, NULL, 0, 0 }
#define string_arg(_name, _mdty)                                                                                       \
  { cl_arg_string, _name, NULL, 0, (_mdty) }
#define int_arg(_name, _mdty)                                                                                          \
  { cl_arg_int, _name, NULL, 0, (_mdty) }
#define real_arg(_name, _mdty)                                                                                         \
  { cl_arg_real, _name, NULL, 0, (_mdty) }
#define strlist_arg(_name, _mdty)                                                                                      \
  { cl_arg_strlist, _name, NULL, 0, (_mdty) }
#define intlist_arg(_name, _mdty)                                                                                      \
  { cl_arg_intlist, _name, NULL, 0, (_mdty) }
#define reallist_arg(_name, _mdty)                                                                                     \
  { cl_arg_reallist, _name, NULL, 0, (_mdty) }
#define sequence_arg(_name, _mdty)                                                                                     \
  { cl_arg_sequence, _name, NULL, 0, (_mdty) }

#ifdef __cplusplus
}
#endif

#endif
