#include "argparse/argparse.h"
#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

typedef enum Error {
  err_none,
  err_out_of_mem,
  err_not_an_int,
  err_int_too_many_dig,
  err_int_out_of_range,
  err_not_a_real_num,
  err_real_too_many_dig,
  err_real_out_of_range,
  err_list_empty_elmt,
  err_invalid_sequence
} Error_t;

void cl_arg_error(const char *msg, int ecode);

void *analyze_string(char *str, Error_t *err, int *size);
void *analyze_int(char *str, Error_t *err, int *size);
void *analyze_real(char *str, Error_t *err, int *size);
void *analyze_strlist(char *str, Error_t *err, int *size);
void *analyze_intlist(char *str, Error_t *err, int *size);
void *analyze_reallist(char *str, Error_t *err, int *size);
void *analyze_sequence(char *str, Error_t *err, int *size);

typedef struct ListElmt {
  void *data;
  struct ListElmt *next; /*"Rekursive Strukturen" in K&R, PiC*/
} ListElmt_t;

typedef struct List {
  ListElmt_t *head;
  ListElmt_t *tail;
  int size;
} List_t;

ListElmt_t *list_elmt_new() {
  ListElmt_t *elmt;

  elmt = (ListElmt_t *)malloc(sizeof(ListElmt_t));
  if (elmt != NULL) {
    elmt->data = NULL;
    elmt->next = NULL;
  }
  return elmt;
}

List_t *list_new() {
  List_t *list;

  list = (List_t *)malloc(sizeof(List_t));
  if (list != NULL) {
    list->head = NULL;
    list->tail = NULL;
    list->size = 0;
  }
  return list;
}

void list_append(List_t *list, ListElmt_t *elmt) {
  ListElmt_t *ptr;

  elmt->next = NULL;

  if (list->head == NULL) {
    list->head = list->tail = elmt;
    list->size = 1;
  } else {
    ptr = list->tail;
    list->tail = elmt;
    ptr->next = elmt;
    list->size += 1;
  }
}

void list_join(List_t *list1, List_t *list2) {
  if (list1->head == NULL) {
    list1->head = list2->head;
    list1->size = list2->size;
  } else {
    if (list2->head != NULL) {
      list1->tail->next = list2->head;
      list1->size += list2->size;
    }
    list2 = NULL;
  }
}

void list_free(List_t *list) {
  ListElmt_t *ptr;

  if (list == NULL)
    return;

  if (list->head != NULL) {
    while (list->head->next != NULL) {
      ptr = list->head;
      list->head = ptr->next;
      if (ptr->data != NULL)
        free(ptr->data);
      free(ptr);
    }
    if (list->tail->data != NULL)
      free(list->tail->data);
    free(list->tail);
  }
  free(list);
}

char msg[257];

void error(Error_t ecode, ClArg_t *arg) {
  switch (ecode) {
  case err_none:
    return;
    break;
  case err_out_of_mem:
    cl_arg_error("Unable to allocate memory", ecode);
    break;
  case err_not_an_int:
    sprintf(msg, "Argument -%c is not an integer", arg->name[0]);
    cl_arg_error(msg, ecode);
    break;
  case err_int_too_many_dig:
  case err_real_too_many_dig:
    sprintf(msg, "Argument -%c has too many digits", arg->name[0]);
    cl_arg_error(msg, ecode);
    break;
  case err_int_out_of_range:
  case err_real_out_of_range:
    sprintf(msg, "Argument -%c is out of range", arg->name[0]);
    cl_arg_error(msg, ecode);
    break;
  case err_not_a_real_num:
    sprintf(msg, "Argument -%c is not a real number", arg->name[0]);
    cl_arg_error(msg, ecode);
    break;
  case err_list_empty_elmt:
    sprintf(msg, "Argument -%c has an empty element", arg->name[0]);
    cl_arg_error(msg, ecode);
    break;
  case err_invalid_sequence:
    sprintf(msg, "Argument -%c is invalid", arg->name[0]);
    cl_arg_error(msg, ecode);
  default:
    cl_arg_error("Unable to parse command", ecode);
  }
}

void cl_arg_error(const char *msg, int ecode) {
  fprintf(stderr, "%s\n", msg);
  exit(ecode);
}

void initialize(ClArg_t *arg) {
  ClArg_t *parg;

  parg = arg;
  while (parg->type != cl_arg_last) {
    parg->value = NULL;
    parg->size = 0;
    parg++;
  }
}

void *analyze_string(char *str, Error_t *err, int *size) {
  void *ptr;
  int len;

  len = strlen(str);
  ptr = malloc((len + 1) * sizeof(char));
  if (ptr != NULL) {
    strcpy((char *)ptr, str);
    *size = len;
    *err = err_none;
  } else {
    *err = err_out_of_mem;
  }

  return ptr;
}

void *analyze_int(char *str, Error_t *err, int *size) {
  const int max_digits = 5;
  const unsigned int max_int = 32767;

  int digit_cnt = 0;
  int neg = 0;
  char *p;
  int val;
  unsigned int uval = 0; /*unsigned, to check if abs. val. fits into signed range*/
  void *ptr;

  p = str;
  if (*p == '-') {
    neg = 1;
    p++;
  }

  while (*p != '\0') {
    if (!isdigit(*p)) {
      *err = err_not_an_int;
      return NULL;
    }
    if (++digit_cnt <= max_digits) {
      uval = 10 * uval + ((*p) - '0');
    } else {
      *err = err_int_too_many_dig;
      return NULL;
    }
    p++;
  }

  if (uval > max_int) {
    *err = err_int_out_of_range;
    return NULL;
  } /* ... now it's safe to cast*/
  val = (int)uval;

  if (neg)
    val = -val;

  ptr = malloc(sizeof(int));
  if (ptr != NULL) {
    *(int *)ptr = val;
    *size = 1;
    *err = err_none;
  } else {
    *err = err_out_of_mem;
  }

  return ptr;
}

/* accumulate part (group of digits) of a real number, reporting errors,
 return a pointer into buf at erroneous char or next char after
 group of digits just read - modified after Mak (1996)*/
char *accumulate(char *buf, int *digit_cnt, double *val, Error_t *err) {
  const int max_digits = 20;

  char *p = buf;

  if (!isdigit(*p)) {
    *err = err_not_a_real_num;
    return p;
  }

  do {
    if (++(*digit_cnt) <= max_digits) {
      *val = 10 * (*val) + ((*p) - '0');
    } else {
      *err = err_real_too_many_dig;
      return p;
    }
    p++;
  } while (isdigit(*p));

  *err = err_none;
  return p;
}

void *analyze_real(char *str, Error_t *err, int *size) {
  /*began module stealing from Mak, R. 1996. Writing compilers and
    interpreters, 2nd ed. Mak uses float for nbr_val, but range of
    float may be exceeded due to ignoring the decimal point -
    used double instead to prevent this, but didn't really check
    if it works for all possible values of a 32-bit float*/

  const int max_exp = 37;

  double nbr_val = 0; /*value of real number ignoring decimal point*/

  int whole_pl = 0;
  int dec_pl = 0;

  char exp_sign = '+';
  double e_val = 0.0; /*exponent part*/
  int expon = 0;

  char *p;
  int neg = 0;
  int digit_cnt = 0;
  void *ptr;

  p = str;
  if (*p == '-') {
    neg = 1;
    p++;
  }

  /*get whole part*/
  if ((*p) != '.') {
    p = accumulate(p, &digit_cnt, &nbr_val, err);
    if ((*err) != err_none)
      return NULL;
    whole_pl = digit_cnt;
  }

  if ((*p) == '.') { /*have fraction part...*/
    p++;
    p = accumulate(p, &digit_cnt, &nbr_val, err);
    if ((*err) != err_none)
      return NULL;
    dec_pl = digit_cnt - whole_pl;
  }

  if (((*p) == 'e') || ((*p) == 'E')) { /*have exponent part...*/
    p++;
    if (((*p) == '+') || ((*p) == '-')) { /*check for exponent sign, if any*/
      exp_sign = *p;
      p++;
    }

    digit_cnt = 0;
    p = accumulate(p, &digit_cnt, &e_val, err);
    if ((*err) != err_none)
      return NULL;

    if (exp_sign == '-')
      e_val = -e_val;
  }

  expon = (int)e_val - dec_pl;
  if ((expon + whole_pl < -max_exp) || (expon + whole_pl > max_exp)) {
    *err = err_real_out_of_range;
    return NULL;
  }

  if (expon != 0)
    nbr_val *= pow(10, expon);

  if (neg)
    nbr_val = -nbr_val;

  ptr = malloc(sizeof(float));
  if (ptr != NULL) {
    *(float *)ptr = (float)nbr_val;
    *size = 1;
    *err = err_none;
  } else {
    *err = err_out_of_mem;
  }

  return ptr;
}

void *analyze_strlist(char *str, Error_t *err, int *size) {
  int len;
  char *buf, *p, *pbuf;
  List_t *list;
  ListElmt_t *elmt;

  len = strlen(str);
  if ((buf = (char *)malloc((len + 1) * sizeof(char))) == NULL) {
    *err = err_out_of_mem;
    return NULL;
  }

  p = str;
  if ((*p) == ',') {
    *err = err_list_empty_elmt;
    free((void *)buf);
    return NULL;
  }
  if ((list = list_new()) == NULL) {
    *err = err_out_of_mem;
    free((void *)buf);
    return NULL;
  }

  pbuf = buf;
  do {
    *(pbuf++) = *(p++);
    if ((*p) == ',') {
      *pbuf = '\0';
      len = strlen(buf);
      if (buf[0] == ',') {
        *err = err_list_empty_elmt;
        free((void *)buf);
        list_free(list);
        return NULL;
      }
      if ((elmt = list_elmt_new()) == NULL) {
        *err = err_out_of_mem;
        free((void *)buf);
        list_free(list);
        return NULL;
      }
      if ((elmt->data = malloc((len + 1) * sizeof(char))) == NULL) {
        *err = err_out_of_mem;
        free((void *)buf);
        list_free(list);
        free((void *)elmt);
        return NULL;
      }
      strcpy((char *)elmt->data, buf);
      list_append(list, elmt);
      pbuf = buf;
      p++;
    }
  } while ((*p) != '\0');

  (*pbuf) = '\0';
  len = strlen(buf);
  if (buf[0] == ',' || len == 0) {
    *err = err_list_empty_elmt;
    free((void *)buf);
    list_free(list);
    return NULL;
  }
  if ((elmt = list_elmt_new()) == NULL) {
    *err = err_out_of_mem;
    free((void *)buf);
    list_free(list);
    return NULL;
  }
  if ((elmt->data = malloc((len + 1) * sizeof(char))) == NULL) {
    *err = err_out_of_mem;
    free((void *)buf);
    list_free(list);
    free((void *)elmt);
    return NULL;
  }
  strcpy((char *)elmt->data, buf);
  list_append(list, elmt);

  free((void *)buf);

  *size = list->size;
  *err = err_none;
  return list;
}

void *analyze_intlist(char *str, Error_t *err, int *size) {
  char *p, *buf, *pbuf;
  List_t *list;
  ListElmt_t *elmt;
  int len;
  void *valptr;

  len = strlen(str);
  if ((buf = (char *)malloc((len + 1) * sizeof(char))) == NULL) {
    *err = err_out_of_mem;
    free((void *)buf);
    return NULL;
  }

  p = str;
  if ((*p) == ',') {
    *err = err_list_empty_elmt;
    free((void *)buf);
    return NULL;
  }
  if ((list = list_new()) == NULL) {
    *err = err_out_of_mem;
    return NULL;
  }

  pbuf = buf;
  do {
    *(pbuf++) = *(p++);
    if ((*p) == ',') {
      *pbuf = '\0';
      if (buf[0] == ',') {
        *err = err_list_empty_elmt;
        free((void *)buf);
        list_free(list);
        return NULL;
      }
      valptr = analyze_int(buf, err, size);
      if (valptr == NULL) {
        free((void *)buf);
        list_free(list);
        return NULL;
      }
      if ((elmt = list_elmt_new()) == NULL) {
        *err = err_out_of_mem;
        free((void *)buf);
        list_free(list);
        return NULL;
      }
      elmt->data = valptr;
      list_append(list, elmt);
      pbuf = buf;
      p++;
    }
  } while ((*p) != '\0');

  (*pbuf) = '\0';
  if (buf[0] == ',' || strlen(buf) == 0) {
    *err = err_list_empty_elmt;
    free((void *)buf);
    list_free(list);
    return NULL;
  }
  if ((elmt = list_elmt_new()) == NULL) {
    *err = err_out_of_mem;
    free((void *)buf);
    list_free(list);
    return NULL;
  }
  valptr = analyze_int(buf, err, size); /*analyze_int sets the error code*/
  if (valptr == NULL) {
    free((void *)buf);
    list_free(list);
    return NULL;
  }
  elmt->data = valptr;
  list_append(list, elmt);

  free((void *)buf);

  *size = list->size;
  *err = err_none;
  return list;
}

/*same code as above, only calls analyze_real instead of analyze_int*/
void *analyze_reallist(char *str, Error_t *err, int *size) {
  char *p, *buf, *pbuf;
  List_t *list;
  ListElmt_t *elmt;
  int len;
  void *valptr;

  len = strlen(str);
  if ((buf = (char *)malloc((len + 1) * sizeof(char))) == NULL) {
    *err = err_out_of_mem;
    free((void *)buf);
    return NULL;
  }

  p = str;
  if ((*p) == ',') {
    *err = err_list_empty_elmt;
    free((void *)buf);
    return NULL;
  }
  if ((list = list_new()) == NULL) {
    *err = err_out_of_mem;
    return NULL;
  }

  pbuf = buf;
  do {
    *(pbuf++) = *(p++);
    if ((*p) == ',') {
      *pbuf = '\0';
      if (buf[0] == ',') {
        *err = err_list_empty_elmt;
        free((void *)buf);
        list_free(list);
        return NULL;
      }
      valptr = analyze_real(buf, err, size); /*analyze_real sets the error code*/
      if (valptr == NULL) {
        free((void *)buf);
        list_free(list);
        return NULL;
      }
      if ((elmt = list_elmt_new()) == NULL) {
        *err = err_out_of_mem;
        free((void *)buf);
        list_free(list);
        return NULL;
      }
      elmt->data = valptr;
      list_append(list, elmt);
      pbuf = buf;
      p++;
    }
  } while ((*p) != '\0');

  (*pbuf) = '\0';
  if (buf[0] == ',' || strlen(buf) == 0) {
    *err = err_list_empty_elmt;
    free((void *)buf);
    list_free(list);
    return NULL;
  }
  if ((elmt = list_elmt_new()) == NULL) {
    *err = err_out_of_mem;
    free((void *)buf);
    list_free(list);
    return NULL;
  }
  valptr = analyze_real(buf, err, size);
  if (valptr == NULL) {
    free((void *)buf);
    list_free(list);
    return NULL;
  }
  elmt->data = valptr;
  list_append(list, elmt);

  free((void *)buf);

  *size = list->size;
  *err = err_none;
  return list;
}

#define OUT 0
#define COLON 1
#define AT 2

void *analyze_sequence(char *str, Error_t *err, int *size) {
  char *p, *buf, *pbuf;
  List_t *list;
  ListElmt_t *elmt;
  int len;
  void *valptr, *valptr2, *valptr3;
  int status;
  int lwr, upr, step, i;
  int dif;

  len = strlen(str);
  if ((buf = (char *)malloc((len + 1) * sizeof(char))) == NULL) {
    *err = err_out_of_mem;
    free((void *)buf);
    return NULL;
  }

  p = str;
  if (!isdigit(*p) && (*p) != '-') {
    free((void *)buf);
    *err = err_invalid_sequence;
    return NULL;
  }

  if ((list = list_new()) == NULL) {
    *err = err_out_of_mem;
    return NULL;
  }

  status = OUT;
  pbuf = buf;
  do {
    *(pbuf++) = *(p++);
    if ((*p) == ',') {
      switch (status) {
      case OUT:                               /*no sequential def., just add single value to sequence*/
        valptr = analyze_int(buf, err, size); /*analyze_int sets the error code*/
        if (valptr == NULL) {
          free((void *)buf);
          list_free(list);
          return NULL;
        }
        if ((elmt = list_elmt_new()) == NULL) {
          *err = err_out_of_mem;
          free((void *)buf);
          free((void *)valptr);
          list_free(list);
          return NULL;
        }
        elmt->data = valptr;
        list_append(list, elmt);
        pbuf = buf;
        p++;
        break;
      case COLON:                              /*sequential def., colon only*/
        valptr2 = analyze_int(buf, err, size); /*analyze_int sets error code*/
        if (valptr2 == NULL) {
          free((void *)buf);
          list_free(list);
          return NULL;
        }
        lwr = *(int *)valptr;
        free(valptr);
        upr = *(int *)valptr2;
        free(valptr2);
        if (lwr == upr) {
          *err = err_invalid_sequence;
          free((void *)buf);
          list_free(list);
          return NULL;
        } else if (lwr > upr) {
          for (i = lwr; i >= upr; i--) {
            if ((elmt = list_elmt_new()) == NULL) {
              *err = err_out_of_mem;
              free((void *)buf);
              list_free(list);
              return NULL;
            }
            if ((elmt->data = malloc(sizeof(int))) == NULL) {
              *err = err_out_of_mem;
              free((void *)buf);
              list_free(list);
              return NULL;
            }
            *(int *)elmt->data = i;
            list_append(list, elmt);
          }
        } else {
          for (i = lwr; i <= upr; i++) {
            if ((elmt = list_elmt_new()) == NULL) {
              *err = err_out_of_mem;
              free((void *)buf);
              list_free(list);
              return NULL;
            }
            if ((elmt->data = malloc(sizeof(int))) == NULL) {
              *err = err_out_of_mem;
              free((void *)buf);
              list_free(list);
              return NULL;
            }
            *(int *)elmt->data = i;
            list_append(list, elmt);
          }
        }
        pbuf = buf;
        p++;
        status = OUT;
        break;
      case AT: /*sequential def. with at sign to modify step size*/
        *pbuf = '\0';
        valptr3 = analyze_int(buf, err, size); /*analyze_int sets error code*/
        if (valptr == NULL) {
          free((void *)buf);
          list_free(list);
          return NULL;
        }
        lwr = *(int *)valptr;
        free(valptr);
        upr = *(int *)valptr2;
        free(valptr2);
        step = *(int *)valptr3;
        free(valptr3);
        if (step == 0) {
          *err = err_invalid_sequence;
          free((void *)buf);
          list_free(list);
          return NULL;
        }
        if (step < 0)
          step = -step;
        if (lwr > upr)
          dif = lwr - upr;
        else
          dif = upr - lwr;
        if ((dif / step) == 0) { /* integer division */
          *err = err_invalid_sequence;
          free((void *)buf);
          list_free(list);
          return NULL;
        }
        if (lwr > upr) {
          for (i = lwr; i >= upr; i -= step) {
            if ((elmt = list_elmt_new()) == NULL) {
              *err = err_out_of_mem;
              free((void *)buf);
              list_free(list);
              return NULL;
            }
            if ((elmt->data = malloc(sizeof(int))) == NULL) {
              *err = err_out_of_mem;
              free((void *)buf);
              list_free(list);
              return NULL;
            }
            *(int *)elmt->data = i;
            list_append(list, elmt);
          }
        } else {
          for (i = lwr; i <= upr; i += step) {
            if ((elmt = list_elmt_new()) == NULL) {
              *err = err_out_of_mem;
              free((void *)buf);
              list_free(list);
              return NULL;
            }
            if ((elmt->data = malloc(sizeof(int))) == NULL) {
              *err = err_out_of_mem;
              free((void *)buf);
              list_free(list);
              return NULL;
            }
            *(int *)elmt->data = i;
            list_append(list, elmt);
          }
        }
        pbuf = buf;
        p++;
        status = OUT;
        break;
      default:
        break;
      }
    } else if ((*p) == ':') {
      *pbuf = '\0';
      valptr = analyze_int(buf, err, size); /*analyze_int sets error code*/
      if (valptr == NULL) {
        free((void *)buf);
        list_free(list);
        return NULL;
      }
      pbuf = buf;
      p++;
      status = COLON;
    } else if ((*p) == '@') {
      if (status != COLON) { /*must have seen a colon before*/
        *err = err_invalid_sequence;
        free((void *)buf);
        list_free(list);
        return NULL;
      }
      *pbuf = '\0';
      valptr2 = analyze_int(buf, err, size); /*analyze_int sets error code*/
      if (valptr2 == NULL) {
        free((void *)buf);
        free(valptr);
        list_free(list);
        return NULL;
      }
      pbuf = buf;
      p++;
      status = AT;
    }
  } while ((*p) != '\0');

  (*pbuf) = '\0';
  if (strlen(buf) == 0) {
    *err = err_invalid_sequence;
    free((void *)buf);
    if (valptr != NULL)
      free(valptr);
    if (valptr2 != NULL)
      free(valptr2);
    if (valptr3 != NULL)
      free(valptr3);
    list_free(list);
    return NULL;
  }

  switch (status) {
  case OUT:                               /*no sequential def., just add single value to sequence*/
    valptr = analyze_int(buf, err, size); /*analyze_int sets the error code*/
    if (valptr == NULL) {
      free((void *)buf);
      list_free(list);
      return NULL;
    }
    if ((elmt = list_elmt_new()) == NULL) {
      *err = err_out_of_mem;
      free((void *)buf);
      free((void *)valptr);
      list_free(list);
      return NULL;
    }
    elmt->data = valptr;
    list_append(list, elmt);
    break;
  case COLON:                              /*sequential def., colon only*/
    valptr2 = analyze_int(buf, err, size); /*analyze_int sets error code*/
    if (valptr2 == NULL) {
      free((void *)buf);
      list_free(list);
      return NULL;
    }
    lwr = *(int *)valptr;
    free(valptr);
    upr = *(int *)valptr2;
    free(valptr2);
    if (lwr == upr) {
      *err = err_invalid_sequence;
      free((void *)buf);
      list_free(list);
      return NULL;
    } else if (lwr > upr) {
      for (i = lwr; i >= upr; i--) {
        if ((elmt = list_elmt_new()) == NULL) {
          *err = err_out_of_mem;
          free((void *)buf);
          list_free(list);
          return NULL;
        }
        if ((elmt->data = malloc(sizeof(int))) == NULL) {
          *err = err_out_of_mem;
          free((void *)buf);
          list_free(list);
          return NULL;
        }
        *(int *)elmt->data = i;
        list_append(list, elmt);
      }
    } else {
      for (i = lwr; i <= upr; i++) {
        if ((elmt = list_elmt_new()) == NULL) {
          *err = err_out_of_mem;
          free((void *)buf);
          list_free(list);
          return NULL;
        }
        if ((elmt->data = malloc(sizeof(int))) == NULL) {
          *err = err_out_of_mem;
          free((void *)buf);
          list_free(list);
          return NULL;
        }
        *(int *)elmt->data = i;
        list_append(list, elmt);
      }
    }
    break;
  case AT:                                 /*sequential def. with at sign to modify step size*/
    valptr3 = analyze_int(buf, err, size); /*analyze_int sets error code*/
    if (valptr == NULL) {
      free((void *)buf);
      list_free(list);
      return NULL;
    }
    lwr = *(int *)valptr;
    free(valptr);
    upr = *(int *)valptr2;
    free(valptr2);
    step = *(int *)valptr3;
    free(valptr3);
    if (step == 0) {
      *err = err_invalid_sequence;
      free((void *)buf);
      list_free(list);
      return NULL;
    }
    if (step < 0)
      step = -step;
    if (lwr > upr)
      dif = lwr - upr;
    else
      dif = upr - lwr;
    if ((dif / step) == 0) { /* integer division */
      *err = err_invalid_sequence;
      free((void *)buf);
      list_free(list);
      return NULL;
    }
    if (lwr > upr) {
      for (i = lwr; i >= upr; i -= step) {
        if ((elmt = list_elmt_new()) == NULL) {
          *err = err_out_of_mem;
          free((void *)buf);
          list_free(list);
          return NULL;
        }
        if ((elmt->data = malloc(sizeof(int))) == NULL) {
          *err = err_out_of_mem;
          free((void *)buf);
          list_free(list);
          return NULL;
        }
        *(int *)elmt->data = i;
        list_append(list, elmt);
      }
    } else {
      for (i = lwr; i <= upr; i += step) {
        if ((elmt = list_elmt_new()) == NULL) {
          *err = err_out_of_mem;
          free((void *)buf);
          list_free(list);
          return NULL;
        }
        if ((elmt->data = malloc(sizeof(int))) == NULL) {
          *err = err_out_of_mem;
          free((void *)buf);
          list_free(list);
          return NULL;
        }
        *(int *)elmt->data = i;
        list_append(list, elmt);
      }
    }
    break;
  default:
    break;
  }

  *size = list->size;
  *err = err_none;
  return list;
}

#undef OUT
#undef COLON
#undef AT

void check_mdty(ClArg_t *arg) {
  ClArg_t *parg;

  parg = arg;
  while (parg->type != cl_arg_last) {
    if (parg->type == cl_arg_pos) {
      if ((parg->mdty == 1) && (parg->size == 0)) {
        sprintf(msg, "Missing operand");
        cl_arg_error(msg, 1);
      }
    } else {
      if ((parg->mdty == 1) && (parg->size == 0)) {
        sprintf(msg, "Option -%c is missing", parg->name[0]);
        cl_arg_error(msg, 1);
      }
    }
    parg++;
  }
}

void parse_cl_args(int argc, char *argv[], ClArg_t *cl_arg) {
  int iv, j, sz;
  ClArg_t *parg, *pmatch;
  char *pv;
  int npos, cpos, c;
  int len;
  int nkey, ikey;
  Error_t err;
  float *preal, *preal2;
  int *pint, *pint2;
  char **papch, **papch2;
  List_t *plist;

  initialize(cl_arg);

  npos = 0;
  parg = cl_arg;
  while (parg->type != cl_arg_last) {
    if (parg->type == cl_arg_pos)
      npos++;
    parg++;
  }

  cpos = 0;

  for (iv = 1; iv < argc; iv++) {
    pv = argv[iv];
    if (pv[0] == '-') { /*option and arg.*/
      if ((len = strlen(pv)) == 1)
        cl_arg_error("Missing \"-\" inserted", 1);
      parg = cl_arg;
      pmatch = NULL;
      while (parg->type != cl_arg_last) {
        if (parg->type != cl_arg_pos) {
          nkey = strlen(parg->name);
          for (ikey = 0; ikey < nkey; ikey++) {
            if (pv[1] == parg->name[ikey]) {
              pmatch = parg;
              break;
            }
          }
        }
        parg++;
      }
      if (pmatch == NULL) {
        sprintf(msg, "Unrecognized option: -%c", pv[1]);
        cl_arg_error(msg, 1);
      }
      if (pmatch->type == cl_arg_flag) { /*simple flag*/
        pmatch->size += 1;
        j = 2;
        while (pv[j] != '\0') { /*grouped flag options*/
          parg = cl_arg;
          pmatch = NULL;
          while (parg->type != cl_arg_last) {
            if (parg->type != cl_arg_pos) {
              nkey = strlen(parg->name);
              for (ikey = 0; ikey < nkey; ikey++) {
                if (pv[j] == parg->name[ikey]) {
                  pmatch = parg;
                  break;
                }
              }
            }
            parg++;
          }
          if (pmatch == NULL) {
            sprintf(msg, "Unrecognized option: -%c", pv[j]);
            cl_arg_error(msg, 1);
          }
          if (pmatch->type == cl_arg_flag) {
            pmatch->size += 1;
          } else {
            sprintf(msg, "Option -%c requires an argument", pv[j]);
            cl_arg_error(msg, 1);
          }
          j++;
        }
      } else {          /*more complex option*/
        if (len == 2) { /*white space between option and value...*/
          iv++;
          if (argc <= iv) {
            sprintf(msg, "Option %s requires an argument", pv);
            cl_arg_error(msg, 1);
          }
          pv = argv[iv];
          if (pv[0] == '-') {
            if (!isdigit(pv[1])) { /*ambiguous! could be negative number...*/
              pv = argv[--iv];
              sprintf(msg, "Option %s requires an argument", pv);
              cl_arg_error(msg, 1);
            }
          }
        } else {
          pv += 2;
        }
        err = err_none;
        switch (pmatch->type) {
        case cl_arg_string:
          if (pmatch->value != NULL) {
            if (pmatch->size > 1) {
              papch = (char **)malloc((pmatch->size + 1) * sizeof(char *));
              papch2 = (char **)pmatch->value;
              for (j = 0; j < pmatch->size; j++) {
                papch[j] = papch2[j];
              }
              papch[pmatch->size] = (char *)analyze_string(pv, &err, &sz);
              pmatch->value = papch;
              pmatch->size += 1;
            } else {
              papch = (char **)malloc(2 * sizeof(char *));
              papch[0] = (char *)pmatch->value;
              papch[1] = (char *)analyze_string(pv, &err, &sz);
              pmatch->value = papch;
              pmatch->size += 1;
            }
          } else {
            pmatch->value = analyze_string(pv, &err, &sz);
            pmatch->size = 1;
          }
          break;
        case cl_arg_int:
          if (pmatch->value != NULL) {
            pint = (int *)pmatch->value;
            sz = pmatch->size;
            pmatch->value = malloc((sz + 1) * sizeof(int));
            if (pmatch->value == NULL)
              error(err_out_of_mem, NULL);
            pint2 = (int *)pmatch->value;
            for (j = 0; j < sz; j++)
              pint2[j] = pint[j];
            free((void *)pint);
            pint = (int *)analyze_int(pv, &err, &(pmatch->size));
            if (pint != NULL) {
              pint2[sz] = *pint;
              free((void *)pint);
              pmatch->size = sz + 1;
            }
          } else {
            pmatch->value = analyze_int(pv, &err, &(pmatch->size));
          }
          break;
        case cl_arg_real:
          if (pmatch->value != NULL) {
            preal = (float *)pmatch->value;
            sz = pmatch->size;
            pmatch->value = malloc((sz + 1) * sizeof(float));
            if (pmatch->value == NULL)
              error(err_out_of_mem, NULL);
            preal2 = (float *)pmatch->value;
            for (j = 0; j < sz; j++)
              preal2[j] = preal[j];
            free((void *)preal);
            preal = (float *)analyze_real(pv, &err, &(pmatch->size));
            if (preal != NULL) {
              preal2[sz] = *preal;
              free((void *)preal);
              pmatch->size = sz + 1;
            }
          } else {
            pmatch->value = analyze_real(pv, &err, &(pmatch->size));
          }
          break;
        case cl_arg_strlist:
          if (pmatch->value != NULL) {
            plist = analyze_strlist(pv, &err, &sz);
            list_join((List_t *)(pmatch->value), plist);
            pmatch->size += sz;
          } else {
            pmatch->value = analyze_strlist(pv, &err, &(pmatch->size));
          }
          break;
        case cl_arg_intlist:
          if (pmatch->value != NULL) {
            plist = analyze_intlist(pv, &err, &sz);
            list_join((List_t *)(pmatch->value), plist);
            pmatch->size += sz;
          } else {
            pmatch->value = analyze_intlist(pv, &err, &(pmatch->size));
          }
          break;
        case cl_arg_reallist:
          if (pmatch->value != NULL) {
            plist = analyze_reallist(pv, &err, &sz);
            list_join((List_t *)(pmatch->value), plist);
            pmatch->size += sz;
          } else {
            pmatch->value = analyze_reallist(pv, &err, &(pmatch->size));
          }
          break;
        case cl_arg_sequence:
          if (pmatch->value != NULL) {
            plist = analyze_sequence(pv, &err, &sz);
            list_join((List_t *)(pmatch->value), plist);
            pmatch->size += sz;
          } else {
            pmatch->value = analyze_sequence(pv, &err, &(pmatch->size));
          }
          break;
        default:
          break;
        }
        if (err != err_none)
          error(err, pmatch);
      }
    } else { /*positional arg.*/
      cpos++;
      if (cpos > npos) {
        sprintf(msg, "Too many operands");
        cl_arg_error(msg, 1);
      }
      parg = cl_arg;
      c = 0;
      while (parg->type != cl_arg_last) {
        if (parg->type == cl_arg_pos) {
          c++;
          if (c == cpos) { /*new positional arg.*/
            parg->value = analyze_string(pv, &err, &(parg->size));
            if (err != err_none)
              error(err, parg);
            break;
          }
        }
        parg++;
      }
    }
  }

  check_mdty(cl_arg);
}

void *get_cl_arg(ClArg_t *arg, const char *name, int *size) {
  ClArg_t *parg;
  List_t *plist;
  ListElmt_t *pelmt;
  char **papch;
  int len, i, j;
  int *pint;
  float *pfloat;

  parg = arg;
  while (parg->type != cl_arg_last) {
    if (strcmp(parg->name, name) == 0) {
      if (parg->size == 0) {
        *size = 0;
        return NULL;
      }
      switch (parg->type) {
      case cl_arg_pos:
      case cl_arg_string:
      case cl_arg_int:
      case cl_arg_real:
        if (parg->size == 0) {
          *size = 0;
          return NULL;
        } else {
          *size = parg->size;
          return parg->value;
        }
      case cl_arg_flag:
        *size = parg->size;
        return NULL;
      case cl_arg_strlist:
        plist = (List_t *)parg->value;
        if ((pelmt = plist->head) == NULL) {
          *size = 0;
          return NULL;
        } else {
          papch = (char **)malloc(plist->size * sizeof(char *));
          if (papch == NULL) {
            *size = -1;
            return NULL;
          }
          i = 0;
          while (pelmt->next != NULL) {
            len = strlen((char *)pelmt->data);
            papch[i] = (char *)malloc(len * sizeof(char));
            if (papch[i] == NULL) {
              for (j = 0; j < i; j++)
                free((void *)papch[j]);
              free((void *)papch);
              *size = -1;
              return NULL;
            }
            strcpy(papch[i], (char *)pelmt->data);
            pelmt = pelmt->next;
            i++;
          }
          len = strlen((char *)pelmt->data);
          papch[i] = (char *)malloc(len * sizeof(char));
          if (papch[i] == NULL) {
            for (j = 0; j < i; j++)
              free((void *)papch[j]);
            free((void *)papch);
            *size = -1;
            return NULL;
          }
          strcpy(papch[i], (char *)pelmt->data);
          *size = plist->size;
          return (void *)papch;
        }
        break;
      case cl_arg_intlist:
      case cl_arg_sequence:
        plist = (List_t *)parg->value;
        if ((pelmt = plist->head) == NULL) {
          *size = 0;
          return NULL;
        } else {
          pint = (int *)malloc(plist->size * sizeof(int));
          if (pint == NULL) {
            *size = -1;
            return NULL;
          }
          i = 0;
          while (pelmt->next != NULL) {
            pint[i] = *(int *)pelmt->data;
            pelmt = pelmt->next;
            i++;
          }
          pint[i] = *(int *)pelmt->data;
          *size = plist->size;
          return (void *)pint;
        }
        break;
      case cl_arg_reallist:
        plist = (List_t *)parg->value;
        if ((pelmt = plist->head) == NULL) {
          *size = 0;
          return NULL;
        } else {
          pfloat = (float *)malloc(plist->size * sizeof(float *));
          if (pfloat == NULL) {
            *size = -1;
            return NULL;
          }
          i = 0;
          while (pelmt->next != NULL) {
            pfloat[i] = *(float *)pelmt->data;
            pelmt = pelmt->next;
            i++;
          }
          pfloat[i] = *(float *)pelmt->data;
          *size = plist->size;
          return (void *)pfloat;
        }
        break;
      default:
        *size = 0;
        return NULL;
        break;
      }
    }
    parg++;
  }

  *size = 0;
  return NULL;
}
