#include <cstdarg>

#include "apps/debug/debug.hpp"
#include "apps/shell/vt100.hpp"
#include "sys/system.hpp"

extern struct sys_impl_s &sys;
extern bool debug_log_enabled;

void debug_app_s::entry(void *args) {
  char **arglist;
  size_t text_len;
  char *cl_buffer_copy;

  cl_buffer_copy = static_cast<char *>(malloc(std::strlen(static_cast<const char *>(args)) + 1u));
  std::memcpy(cl_buffer_copy, static_cast<const char *>(args), std::strlen(static_cast<const char *>(args)) + 1u);

  // Do some stuff
  if (!(arglist = split_string_(cl_buffer_copy, " "))) {
    goto exit;
  }

  if (arglist[1u]) {
    if (!std::strcmp(arglist[1u], "enable")) {

	  debug_log_enabled = true;
      debug_printfmt("Enabled!\r\n");
    } else if (!std::strcmp(arglist[1u], "disable")) {
	  
      debug_log_enabled = false;
      debug_printfmt("Disabled!\r\n");
	  
    } else {
      debug_printfmt("Error!\r\n");
    }
  }

exit:
  free(cl_buffer_copy);
}

char **debug_app_s::split_string_(char *str, const char *delim) {
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

int32_t debug_app_s::debug_printfmt(const char *fmt, ...) {
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

    if ((rc = ::write(usart, usart_fd, "[debug] : ", std::strlen("[debug] : "))) < 0) {
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
