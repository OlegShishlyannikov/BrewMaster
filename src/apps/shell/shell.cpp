#include <cstdarg>

#include "apps/shell/shell.hpp"
#include "apps/shell/vt100.hpp"
#include "sys/system.hpp"

extern struct sys_impl_s &sys;

static uint32_t cursor_x_pos = 0u;
static uint32_t cursor_y_pos = 0u;
static enum shell_fsm_state_e state;
static char shell_cl_buffer[128u]{0x00};
static char shell_promt_buffer[128u]{0x00};

void shell_app_s::entry(void *cl_args) {
  const char *args = *static_cast<const char **>(cl_args);
  size_t arg_str_len = std::strlen(args), optind;
  char *argsbuffer = static_cast<char *>(malloc(arg_str_len)), *p;
  char **argv = nullptr;
  int32_t argc = 0;

  int32_t console_fd, rc;
  struct usart_callback_req_s console_cbk_req {
    .callback = console_char_recvd_callback
  };

  static const char *uname = "root";
  static const char *system_name = "brewmaster_controller";
  static const char *homedir_name = "/";

  // Split string to args
  std::strncpy(argsbuffer, args, arg_str_len);
  p = std::strtok(argsbuffer, " ");

  while (p) {
    argc++;
    argv = static_cast<char **>(realloc(argv, sizeof(char **) * argc));
    argv[argc - 1u] = p;
    p = std::strtok(nullptr, " ");
  }

  // No args to parse
  free(argv);

  // Set console char recvd callback
  if ((console_fd = ::open(&sys, "usart/usart1", 3, 2u)) < 0) {
    goto error;
  }

  if ((rc = ::ioctl(&sys, "usart/usart1", usart_ioctl_cmd_e::USART_ON_RECV, &console_cbk_req, sizeof(console_cbk_req))) < 0) {
    if ((rc = ::close(&sys, "usart/usart1")) < 0) {
      goto error;
    }
    goto error;
  }

  if ((rc = ::close(&sys, "usart/usart1")) < 0) {
    goto error;
  }

  // Reset FSM state
  state = SHELL_APP_FSM_STATE_STARTUP;
  std::sprintf(shell_promt_buffer, "[%s@%s %s] # ", uname, system_name, homedir_name);

  // Loop -- do nothing, working on callbacks
  while (true) {
    vTaskDelay(1000u);
  }

error:
  vTaskDelete(nullptr);
}

void shell_app_s::console_char_recvd_callback(const void *data, size_t size) {
  const char recvd_char = *static_cast<const char *>(data);
  int32_t rc, usart_fd;

  switch (state) {
  case SHELL_APP_FSM_STATE_STARTUP: {
    // Print welcome screen
    shell_printfmt("\r\nEnter root password : \r\n");
    update_cursor_pos_(&cursor_x_pos, &cursor_y_pos, nullptr);
    state = SHELL_APP_FSM_STATE_WAITING;
  } break;

  case SHELL_APP_FSM_STATE_WAITING: {
    static constexpr const uint64_t pass_hash = hash_64_fnv1a_const("brewmaster");
    static char password_buffer[16u]{0x00};

    // Wait for password
    if (recvd_char == ASCII_CTRL_CH_CR || recvd_char == ASCII_CTRL_CH_LF) {

      // Check password on enter key pressed
      if (hash_64_fnv1a(password_buffer) == pass_hash) {
        shell_printfmt("%s%s", ASCII_CONTROL_ERASE_LINE, ASCII_CONTROL_CURSOR_LINE_BEGIN);
        shell_printfmt("======================================================================================================\r\n"
                       "=  ====  ====  ==        ==  ==========     =====    ====  =====  ==        =======        ====    ===\r\n"
                       "=  ====  ====  ==  ========  =========  ===  ===  ==  ===   ===   ==  ================  ======  ==  ==\r\n"
                       "=  ====  ====  ==  ========  ========  ========  ====  ==  =   =  ==  ================  =====  ====  =\r\n"
                       "=  ====  ====  ==  ========  ========  ========  ====  ==  == ==  ==  ================  =====  ====  =\r\n"
                       "=   ==    ==  ===      ====  ========  ========  ====  ==  =====  ==      ============  =====  ====  =\r\n"
                       "==  ==    ==  ===  ========  ========  ========  ====  ==  =====  ==  ================  =====  ====  =\r\n"
                       "==  ==    ==  ===  ========  ========  ========  ====  ==  =====  ==  ================  =====  ====  =\r\n"
                       "===    ==    ====  ========  =========  ===  ===  ==  ===  =====  ==  ================  ======  ==  ==\r\n"
                       "====  ====  =====        ==        ====     =====    ====  =====  ==        ==========  =======    ===\r\n"
                       "======================================================================================================\r\n"
                       "=============================================================================================================="
                       "=\r\n"
                       "=      ====       ===        ==  ====  ====  ==  =====  =====  ======      ===        ==        ==       ===  "
                       "=\r\n"
                       "=  ===  ===  ====  ==  ========  ====  ====  ==   ===   ====    ====  ====  =====  =====  ========  ====  ==  "
                       "=\r\n"
                       "=  ====  ==  ====  ==  ========  ====  ====  ==  =   =  ===  ==  ===  ====  =====  =====  ========  ====  ==  "
                       "=\r\n"
                       "=  ===  ===  ===   ==  ========  ====  ====  ==  == ==  ==  ====  ===  ==========  =====  ========  ===   ==  "
                       "=\r\n"
                       "=      ====      ====      ====   ==    ==  ===  =====  ==  ====  =====  ========  =====      ====      ====  "
                       "=\r\n"
                       "=  ===  ===  ====  ==  =========  ==    ==  ===  =====  ==        =======  ======  =====  ========  ====  ==  "
                       "=\r\n"
                       "=  ====  ==  ====  ==  =========  ==    ==  ===  =====  ==  ====  ==  ====  =====  =====  ========  ====  "
                       "=====\r\n"
                       "=  ===  ===  ====  ==  ==========    ==    ====  =====  ==  ====  ==  ====  =====  =====  ========  ====  ==  "
                       "=\r\n"
                       "=      ====  ====  ==        =====  ====  =====  =====  ==  ====  ===      ======  =====        ==  ====  ==  "
                       "=\r\n"
                       "=============================================================================================================="
                       "=\r\n");

        shell_printfmt(shell_promt_buffer);
        update_cursor_pos_(&cursor_x_pos, &cursor_y_pos, nullptr);
        state = SHELL_APP_FSM_STATE_IDLE;
      }

      // Clear buffer if typed password invalid
      std::memset(password_buffer, '\0', sizeof(password_buffer));
      update_cursor_pos_(&cursor_x_pos, &cursor_y_pos, nullptr);

      // If password has being typed
    } else if (recvd_char >= 0x20 && recvd_char <= 0x7e) {
      append_char_(password_buffer, &recvd_char, sizeof(password_buffer));
    }
  } break;

  case SHELL_APP_FSM_STATE_IDLE: {
    switch (recvd_char) {
      // If crtl char recvd
    case ASCII_CTRL_CH_NUL ... ASCII_CTRL_CH_US: {

      // CR | LF -- handle buffer and goto idle state
      if (recvd_char == ASCII_CTRL_CH_CR || recvd_char == ASCII_CTRL_CH_LF) {

        handle_eol_char_(recvd_char);

        // ESC character
      } else if (recvd_char == ASCII_CTRL_CH_ESC) {

        handle_esc_seq_(recvd_char);

        // Backspace character
      } else if (recvd_char == ASCII_CTRL_CH_BS) {

        handle_ctrl_char_(recvd_char);
      }
    } break;

    case ASCII_CTRL_CH_DEL: {
      handle_ctrl_char_(recvd_char);
    } break;

      // Generic ASCII symbols
    case 0x20 ... 0x7e: {
      handle_generic_ascii_(recvd_char);
    } break;

    default:
      break;
    }
  } break;

  case SHELL_APP_FSM_STATE_ESC_SEQ_HANDLING: {
    handle_esc_seq_(recvd_char);
  } break;

  case SHELL_APP_FSM_STATE_CURSOR_POS_UPDATING: {
    update_cursor_pos_(&cursor_x_pos, &cursor_y_pos, &recvd_char);
  } break;

  default:
    break;
  }

  return;
error:
  return;
}

// Execute other application
int32_t shell_app_s::exec(const char *app_name, int app_argc, char **app_argv) {
  // const struct app_s app_entry = sys.app("name");
  // return app_entry.entry(app_argc, app_argv);
  return 0;
error:
  return -1;
}

int32_t shell_app_s::shell_printfmt(const char *fmt, ...) {
  int32_t rc, usart_fd, strlen;
  static char *temp;
  const struct drv_model_cmn_s *usart;

  std::va_list arg;
  va_start(arg, fmt);
  size_t bufsz = std::vsnprintf(nullptr, 0u, fmt, arg);
  temp = static_cast<char *>(calloc(bufsz, sizeof(char)));
  strlen = std::vsprintf(temp, fmt, arg);
  va_end(arg);
  
  if (!(usart = sys.drv("usart"))) {
    goto error;
  }

  if (strlen) {
    if ((usart_fd = ::open(usart, "usart1", 3, 3u)) < 0) {
      goto error;
    }

    if ((rc = ::write(usart, usart_fd, temp, std::strlen(temp))) < 0) {
      if ((rc = ::close(usart, usart_fd)) < 0) {
        goto error;
      }

      goto error;
    }

    if ((rc = ::close(usart, usart_fd)) < 0) {
      goto error;
    }
  }

 exit:
  free(temp);
  return strlen;
error:

  free(temp);
  return -1;
}

bool shell_app_s::starts_with_(const char *str, const char *pfx, size_t lenpfx) {
  if (!str || !pfx) {
    return false;
  }

  size_t lenstr = strlen(str);

  if (lenpfx > lenstr) {
    return false;
  }

  return !std::strncmp(str, pfx, lenpfx);
}

bool shell_app_s::ends_with_(const char *str, const char *sfx, size_t lensfx) {
  if (!str || !sfx) {
    return false;
  }

  size_t lenstr = strlen(str);

  if (lensfx > lenstr) {
    return false;
  }

  return !std::strncmp(str + lenstr - lensfx, sfx, lensfx);
}

char **shell_app_s::split_string_(char *str, const char *delim) {
  char **res = nullptr, *p;
  int32_t n = 0;

  if (!str || !delim) {
    return nullptr;
  }

  p = std::strtok(str, delim);

  while (p) {
    n++;
    res = static_cast<char **>(realloc(res, sizeof(char **) * n));
    res[n - 1u] = p;
    p = std::strtok(nullptr, " ");
  }

  // Terminate list
  res = static_cast<char **>(realloc(res, sizeof(char **) * n + 1u));
  res[n] = nullptr;
  return res;
}

char *shell_app_s::append_char_(char *str, const char *chr, size_t maxlen) {
  size_t strlen = std::strlen(str);
  char *res;

  if (strlen >= (maxlen - 1u)) {
    goto error;
  }

  if (!(res = std::strncat(str, chr, sizeof(char)))) {
    goto error;
  }

  return res;
error:
  return nullptr;
}

char *shell_app_s::insert_char_(char *str, const char *chr, size_t npos, size_t maxlen) {
  size_t strlen = std::strlen(str);
  void *res;

  // If buffer filled
  if (strlen == maxlen) {
    if (!(res = std::memmove(str + npos + 1u, str + npos, std::strlen(str + npos)))) {
      goto error;
    }

    str[maxlen - 1u] = '\0';
    *(str + npos) = *chr;
  } else if (strlen < maxlen) {
    if (!(res = std::memmove(str + npos + 1u, str + npos, std::strlen(str + npos) + 1u))) {
      goto error;
    }

    *(str + npos) = *chr;
  } else {
    goto error;
  }

  return static_cast<char *>(res);
error:
  return nullptr;
}

char *shell_app_s::remove_char_left_(char *str, size_t npos) {
  size_t strlen = std::strlen(str);
  void *res;

  // No characters left 0 position
  if (!npos) {
    goto error;
  }

  if (!(res = std::memmove(str + npos - 1u, str + npos, std::strlen(str + npos)))) {
    goto error;
  }

  str[strlen - 1u] = '\0';
  return static_cast<char *>(res);
error:
  return nullptr;
}

char *shell_app_s::remove_char_right_(char *str, size_t npos) {
  size_t strlen = std::strlen(str);
  void *res;

  // No characters right last char position
  if (npos == strlen) {
    goto error;
  }

  if (!(res = std::memmove(str + npos, str + npos + 1u, std::strlen(str + npos)))) {
    goto error;
  }

  str[strlen - 1u] = '\0';
  return static_cast<char *>(res);
error:
  return nullptr;
}

bool shell_app_s::update_cursor_pos_(uint32_t *pos_x, uint32_t *pos_y, const char *recvd_char) {
  int32_t rc;
  static char esc_seq_buffer[10u];
  static uint32_t esc_seq_cnt = 0;

  switch (state) {
  case SHELL_APP_FSM_STATE_CURSOR_POS_UPDATING: {
    if (!recvd_char) {
      return false;
    }

    // Escape sequence handling
    esc_seq_buffer[esc_seq_cnt++] = *recvd_char;
    if (const char *report = ASCII_CONTROL_REPORT_CURSOR_POS; starts_with_(esc_seq_buffer, report, 2u) && ends_with_(esc_seq_buffer, &report[std::strlen(report) - 1u], 1u)) {

      // Sequence received
      esc_seq_cnt = 0u;

      uint32_t x, y;
      if (std::sscanf(esc_seq_buffer, ASCII_CONTROL_REPORT_CURSOR_POS, &y, &x) == 2u) {

        *pos_x = x;
        *pos_y = y;

        // Got position!
        std::memset(esc_seq_buffer, '\0', sizeof(esc_seq_buffer));
        state = SHELL_APP_FSM_STATE_IDLE;
        return true;
      }

      // Failed! Try again
      std::memset(esc_seq_buffer, '\0', sizeof(esc_seq_buffer));
      return false;
    } else if (esc_seq_cnt == sizeof(esc_seq_buffer)) {

      state = SHELL_APP_FSM_STATE_IDLE;
      esc_seq_cnt = 0u;
      std::memset(esc_seq_buffer, '\0', sizeof(esc_seq_buffer));
    }

    return false;
  } break;

  case SHELL_APP_FSM_STATE_ESC_SEQ_HANDLING: {
  } break;

  case SHELL_APP_FSM_STATE_IDLE: {
    state = SHELL_APP_FSM_STATE_CURSOR_POS_UPDATING;
    shell_printfmt(ASCII_CONTROL_QUERY_CURSOR_POS);

    // Waiting other char
    return false;
  } break;

  default:
    break;
  }

  return false;
}

void shell_app_s::handle_esc_seq_(const char recvd) {
  int32_t rc;
  static char esc_seq_buffer[10u];
  static uint32_t esc_seq_cnt = 0;

  switch (state) {
  case SHELL_APP_FSM_STATE_ESC_SEQ_HANDLING: {
    if (!recvd) {
      return;
    }

    // Escape sequence handling
    esc_seq_buffer[esc_seq_cnt++] = recvd;
    if (std::strstr(esc_seq_buffer, ASCII_CONTROL_CURSOR_HOME)) {

      update_cursor_pos_(&cursor_x_pos, &cursor_y_pos, nullptr);
      shell_printfmt(ASCII_CONTROL_CURSOR_HOME);
      std::memset(esc_seq_buffer, '\0', sizeof(esc_seq_buffer));
      esc_seq_cnt = 0u;
      state = SHELL_APP_FSM_STATE_IDLE;
      update_cursor_pos_(&cursor_x_pos, &cursor_y_pos, nullptr);
    } else if (std::strstr(esc_seq_buffer, ASCII_CONTROL_CURSOR_LINE_BEGIN)) {

      update_cursor_pos_(&cursor_x_pos, &cursor_y_pos, nullptr);
      shell_printfmt(ASCII_CONTROL_SET_CURSOR_POS, cursor_y_pos, std::strlen(shell_promt_buffer) + 1u);
      std::memset(esc_seq_buffer, '\0', sizeof(esc_seq_buffer));
      esc_seq_cnt = 0u;
      state = SHELL_APP_FSM_STATE_IDLE;
      update_cursor_pos_(&cursor_x_pos, &cursor_y_pos, nullptr);
    } else if (std::strstr(esc_seq_buffer, ASCII_CONTROL_INS)) {

      update_cursor_pos_(&cursor_x_pos, &cursor_y_pos, nullptr);
      shell_printfmt(ASCII_CONTROL_INS);
      std::memset(esc_seq_buffer, '\0', sizeof(esc_seq_buffer));
      esc_seq_cnt = 0u;
      state = SHELL_APP_FSM_STATE_IDLE;
      update_cursor_pos_(&cursor_x_pos, &cursor_y_pos, nullptr);
    } else if (std::strstr(esc_seq_buffer, ASCII_CONTROL_DELETE)) {

      update_cursor_pos_(&cursor_x_pos, &cursor_y_pos, nullptr);
      shell_printfmt(ASCII_CONTROL_DELETE);
      std::memset(esc_seq_buffer, '\0', sizeof(esc_seq_buffer));
      esc_seq_cnt = 0u;
      state = SHELL_APP_FSM_STATE_IDLE;
      update_cursor_pos_(&cursor_x_pos, &cursor_y_pos, nullptr);
    } else if (std::strstr(esc_seq_buffer, ASCII_CONTROL_CURSOR_LINE_END)) {

      update_cursor_pos_(&cursor_x_pos, &cursor_y_pos, nullptr);
      shell_printfmt(ASCII_CONTROL_SET_CURSOR_POS, cursor_y_pos, std::strlen(shell_cl_buffer) + std::strlen(shell_promt_buffer) + 1u);
      std::memset(esc_seq_buffer, '\0', sizeof(esc_seq_buffer));
      esc_seq_cnt = 0u;
      state = SHELL_APP_FSM_STATE_IDLE;
      update_cursor_pos_(&cursor_x_pos, &cursor_y_pos, nullptr);
    } else if (std::strstr(esc_seq_buffer, ASCII_CONTROL_CURSOR_FORWARD)) {

      update_cursor_pos_(&cursor_x_pos, &cursor_y_pos, nullptr);

      if (cursor_x_pos < std::strlen(shell_cl_buffer) + std::strlen(shell_promt_buffer) + 1u) {
        shell_printfmt(ASCII_CONTROL_CURSOR_FORWARD);
      }

      std::memset(esc_seq_buffer, '\0', sizeof(esc_seq_buffer));
      esc_seq_cnt = 0u;
      state = SHELL_APP_FSM_STATE_IDLE;
      update_cursor_pos_(&cursor_x_pos, &cursor_y_pos, nullptr);
    } else if (std::strstr(esc_seq_buffer, ASCII_CONTROL_CURSOR_BACKWARD)) {

      update_cursor_pos_(&cursor_x_pos, &cursor_y_pos, nullptr);

      if (cursor_x_pos > std::strlen(shell_promt_buffer) + 1u) {
        shell_printfmt(ASCII_CONTROL_CURSOR_BACKWARD);
      }

      std::memset(esc_seq_buffer, '\0', sizeof(esc_seq_buffer));
      esc_seq_cnt = 0u;
      state = SHELL_APP_FSM_STATE_IDLE;
      update_cursor_pos_(&cursor_x_pos, &cursor_y_pos, nullptr);
    } else if (std::strstr(esc_seq_buffer, ASCII_CONTROL_KBD_CURSOR_CTRL_FORWARD)) {

      update_cursor_pos_(&cursor_x_pos, &cursor_y_pos, nullptr);
      std::memset(esc_seq_buffer, '\0', sizeof(esc_seq_buffer));
      esc_seq_cnt = 0u;
      state = SHELL_APP_FSM_STATE_IDLE;
      update_cursor_pos_(&cursor_x_pos, &cursor_y_pos, nullptr);
    } else if (std::strstr(esc_seq_buffer, ASCII_CONTROL_KBD_CURSOR_CTRL_BACKWARD)) {

      update_cursor_pos_(&cursor_x_pos, &cursor_y_pos, nullptr);
      std::memset(esc_seq_buffer, '\0', sizeof(esc_seq_buffer));
      esc_seq_cnt = 0u;
      state = SHELL_APP_FSM_STATE_IDLE;
      update_cursor_pos_(&cursor_x_pos, &cursor_y_pos, nullptr);
    } else {
      if (esc_seq_cnt == sizeof(esc_seq_buffer)) {

        update_cursor_pos_(&cursor_x_pos, &cursor_y_pos, nullptr);
        std::memset(esc_seq_buffer, '\0', sizeof(esc_seq_buffer));
        esc_seq_cnt = 0u;
        state = SHELL_APP_FSM_STATE_IDLE;
      }
    }
  } break;

  case SHELL_APP_FSM_STATE_IDLE: {
    state = SHELL_APP_FSM_STATE_ESC_SEQ_HANDLING;
    esc_seq_buffer[esc_seq_cnt++] = recvd;
  } break;

  case SHELL_APP_FSM_STATE_CURSOR_POS_UPDATING: {
  } break;

  default:
    break;
  }
}

void shell_app_s::handle_ctrl_char_(const char recvd) {
  if (recvd == ASCII_CTRL_CH_BS || recvd == ASCII_CTRL_CH_DEL) {
    remove_char_left_(shell_cl_buffer, cursor_x_pos - std::strlen(shell_promt_buffer) - 1u);
    shell_printfmt(ASCII_CONTROL_ERASE_LINE);
    shell_printfmt(ASCII_CONTROL_SET_CURSOR_POS, cursor_y_pos, 0u);
    shell_printfmt(shell_promt_buffer);
    shell_printfmt("%s", shell_cl_buffer);

    if (cursor_x_pos - std::strlen(shell_promt_buffer) - 1u) {
      cursor_x_pos--;
    }

    shell_printfmt(ASCII_CONTROL_SET_CURSOR_POS, cursor_y_pos, cursor_x_pos);
    update_cursor_pos_(&cursor_x_pos, &cursor_y_pos, nullptr);
  }
}

void shell_app_s::handle_eol_char_(const char recvd) {
  char **arglst;
  static char cl_buffer_copy[sizeof(shell_cl_buffer)];

  std::memcpy(cl_buffer_copy, shell_cl_buffer, sizeof(shell_cl_buffer));

  if ((arglst = split_string_(cl_buffer_copy, " "))) {
    char *appname = arglst[0];
    struct app_s *app = sys.app(appname);
    if (app) {
      app->entry(shell_cl_buffer);
    } else {
      if (std::strlen(shell_cl_buffer)) {
        shell_printfmt("\r\nUnknown command \"%s\"\r\n", shell_cl_buffer);
      }
    }

    free(arglst);
  }

  shell_printfmt("\r\n");
  shell_printfmt(shell_promt_buffer);
  std::memset(shell_cl_buffer, '\0', sizeof(shell_cl_buffer));
  update_cursor_pos_(&cursor_x_pos, &cursor_y_pos, nullptr);
}

void shell_app_s::handle_generic_ascii_(const char recvd) {
  size_t buf_len = std::strlen(shell_cl_buffer);

  if (((cursor_x_pos - std::strlen(shell_promt_buffer)) > buf_len || !buf_len) && buf_len < sizeof(shell_cl_buffer) - 1u) {

    if (append_char_(shell_cl_buffer, &recvd, sizeof(shell_cl_buffer))) {
      shell_printfmt(ASCII_CONTROL_ERASE_LINE);
      shell_printfmt(ASCII_CONTROL_SET_CURSOR_POS, cursor_y_pos, 0u);
      shell_printfmt(shell_promt_buffer);
      shell_printfmt("%s", shell_cl_buffer);
      cursor_x_pos++;
    }
  } else if (((cursor_x_pos - std::strlen(shell_promt_buffer)) <= buf_len) && buf_len < sizeof(shell_cl_buffer) - 1u) {

    if (insert_char_(shell_cl_buffer, &recvd, cursor_x_pos - std::strlen(shell_promt_buffer) - 1u, sizeof(shell_cl_buffer))) {
      shell_printfmt(ASCII_CONTROL_ERASE_LINE);
      shell_printfmt(ASCII_CONTROL_SET_CURSOR_POS, cursor_y_pos, 0u);
      shell_printfmt(shell_promt_buffer);
      shell_printfmt("%s", shell_cl_buffer);
      shell_printfmt(ASCII_CONTROL_SET_CURSOR_POS, cursor_y_pos, cursor_x_pos + 1u);
      cursor_x_pos++;
    }
  }
}

void shell_app_s::handle_kbd_(const char) {}
