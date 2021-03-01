#include <cstdarg>

#include "apps/cowsay/cowsay.hpp"
#include "apps/shell/vt100.hpp"
#include "sys/system.hpp"

extern struct sys_impl_s &sys;
extern bool debug_log_enabled;

void cowsay_app_s::entry(void *args) {
  char **arglist;
  char *cl_buffer_copy;
  const char *text = "What???";
  size_t text_len;

  cl_buffer_copy = static_cast<char *>(malloc(std::strlen(static_cast<const char *>(args)) + 1u));
  std::memcpy(cl_buffer_copy, static_cast<const char *>(args), std::strlen(static_cast<const char *>(args)) + 1u);

  if ((arglist = split_string_(cl_buffer_copy, " "))) {
    if (arglist[1u]) {
      text = arglist[1u];
    }
  }

  text_len = std::strlen(text);
  printfmt("\r\n");

  for (uint32_t i = 0u; i < text_len + 2u; i++) {
    printfmt("%c", ASCII_EXT_CH_HD_DOTTED_GCH);
  }

  printfmt("\r\n");
  printfmt("%c%s%c\r\n", ASCII_EXT_CH_HD_DOTTED_GCH, text, ASCII_EXT_CH_HD_DOTTED_GCH);

  for (uint32_t i = 0u; i < text_len + 2u; i++) {
    printfmt("%c", ASCII_EXT_CH_HD_DOTTED_GCH);
  }

  printfmt("\r\n");
  printfmt("     \\  ^___^\r\n");
  printfmt("      \\ (ooo)\\_______\r\n");
  printfmt("        (___)\\       )~~~\r\n");
  printfmt("             ||----w |\r\n");
  printfmt("             ||     ||\r\n");

exit:
  free(cl_buffer_copy);
}

char **cowsay_app_s::split_string_(char *str, const char *delim) {
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

int32_t cowsay_app_s::printfmt(const char *fmt, ...) {
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

  free(temp);
  return strlen;
error:

  free(temp);
  return -1;
}

int32_t cowsay_app_s::cowsay_printfmt(const char *fmt, ...) {
  int32_t rc, usart_fd, strlen;
  static char *temp;
  const struct drv_model_cmn_s *usart;

  std::va_list arg;
  va_start(arg, fmt);
  size_t bufsz = std::vsnprintf(nullptr, 0u, fmt, arg);
  temp = static_cast<char *>(calloc(bufsz, sizeof(char)));
  strlen = std::vsprintf(temp, fmt, arg);
  va_end(arg);

  if (!debug_log_enabled) {
    goto exit;
  }

  if (!(usart = sys.drv("usart"))) {
    goto error;
  }

  if (strlen) {
    if ((usart_fd = ::open(usart, "usart1", 3, 3u)) < 0) {
      goto error;
    }

    if ((rc = ::write(usart, usart_fd, "[cowsay] : ", std::strlen("[cowsay] : "))) < 0) {
      if ((rc = ::close(usart, usart_fd)) < 0) {
        goto error;
      }
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
