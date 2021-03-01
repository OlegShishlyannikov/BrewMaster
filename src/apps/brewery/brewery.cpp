#include <cstdarg>
#include <malloc.h>

#include "apps/brewery/brewery.hpp"
#include "apps/shell/vt100.hpp"
#include "lfs/lfs.h"
#include "sys/system.hpp"
#include "util/hash/sha256.hpp"
#include "json/json.h"

extern struct sys_impl_s &sys;

extern int32_t lfs_config();
extern lfs_t lfs;
extern lfs_file_t file;
extern struct lfs_config lfs_cfg;
extern bool debug_log_enabled;

static constexpr const uint32_t heater_ac_load_num = 2u;

static xSemaphoreHandle save_pending_semphr, menu_semphr, menu_variants_semphr, thermometer_proc_sem, brewery_proc_sem, temp_hold_proc_sem, temp_hold_timer_proc_sem;

// Menu entry local variables
static struct jfes_value *menu;
static struct jfes_value *menu_variants;
static struct jfes_value *current_menu_entry_ptr;
static struct jfes_value *cursor_entry_ptr;
static struct jfes_value *pending_to_save;
static struct jfes_value *brewery_recipe;

static constexpr const uint32_t lcd_lines_num = 4u;
static constexpr const uint32_t lcd_columns_num = 20u;
static constexpr const uint32_t lcd_fb_x_size = lcd_columns_num * 2u + 1u; // Additional 1 for null terminated strings
static constexpr const uint32_t lcd_fb_y_size = lcd_lines_num * 4u;
static constexpr const uint32_t lcd_fb_window_x_size = lcd_columns_num;
static constexpr const uint32_t lcd_fb_window_y_size = lcd_lines_num;

static int32_t mount_fs();

// JSON file utils
static struct jfes_value *generate_default_menu();
static struct jfes_value *generate_default_menu_variants();
static struct jfes_value *get_json_value(const char *, struct jfes_value *(*)(), sha256::sha256_hash_type *);
static struct jfes_value *read_value_from_file(const char *, struct lfs_info *, sha256::sha256_hash_type *);
static struct jfes_value *write_value_to_file(const char *, struct jfes_value *, sha256::sha256_hash_type *);
static void print_sha256(const sha256::sha256_hash_type *);

// LCD utils
// using lcd_fb_ta = char **[lcd_fb_y_size][lcd_fb_x_size];
using lcd_fb_ta = char[lcd_fb_y_size][lcd_fb_x_size];
using lcd_fb_window_ta = char (*[lcd_fb_window_y_size])[lcd_fb_window_x_size];

static lcd_fb_ta lcd_fb;

static void lcd_console_init();
static void lcd_console_deinit();
static void lcd_clear();
static void lcd_clear_line(uint8_t);
static void lcd_fill_line(uint8_t, char);
static void lcd_print_line(char (**)[lcd_fb_window_x_size], uint8_t);
static void lcd_flush_fb_window(lcd_fb_window_ta *);
static void lcd_set_cursor(uint8_t, uint8_t);
static void lcd_enable_blinking();
static void lcd_disable_blinking();

// LCD Framebuffer utils
static void lcd_fb_clear(lcd_fb_ta *);
static void lcd_fb_clear_line(lcd_fb_ta *, uint8_t);
static void lcd_fb_fill_line(lcd_fb_ta *, uint8_t, char);
static void lcd_fb_print_line(lcd_fb_ta *, uint8_t, const char (**)[lcd_fb_x_size], enum lcd_align_e = LCD_ALIGN_CENTER);
static void lcd_get_window(lcd_fb_ta *, lcd_fb_window_ta *, uint8_t, uint8_t, enum lcd_align_e = LCD_ALIGN_CENTER);

// JSON to menu utils
static void trigger_json_val(lcd_fb_ta *, struct jfes_value *, const char *, enum lcd_align_e = LCD_ALIGN_CENTER);
static void set_cursor_position(enum lcd_align_e = LCD_ALIGN_CENTER);
static void unset_cursor_position(enum lcd_align_e = LCD_ALIGN_CENTER);

// Button utilities
static int32_t init_button_callbacks();
static int32_t deinit_button_callbacks();
static void up_button_callback(const void *, size_t);
static void down_button_callback(const void *, size_t);
static void set_button_callback(const void *, size_t);
static void down_short_push_callback();
static void up_short_push_callback();
static void down_long_push_callback();
static void up_long_push_callback();
static void set_short_push_callback();
static void set_long_push_callback();

// Load manage utilites
static void ac_load_on(int32_t);
static void ac_load_off(int32_t);
static void ac_load_switch(int32_t);

static void relay_load_on(int32_t);
static void relay_load_off(int32_t);
static void relay_load_switch(int32_t);

// Temperature conversion utilites
static int32_t celsius_to_kelvin(int32_t);
static int32_t celsius_to_fahrenheit(int32_t);
static int32_t kelvin_to_celsius(int32_t);
static int32_t kelvin_to_fahrenheit(int32_t);
static int32_t fahrenheit_to_celsius(int32_t);
static int32_t fahrenheit_to_kelvin(int32_t);

static double celsius_to_kelvin_dbl(double);
static double celsius_to_fahrenheit_dbl(double);
static double kelvin_to_celsius_dbl(double);
static double kelvin_to_fahrenheit_dbl(double);
static double fahrenheit_to_celsius_dbl(double);
static double fahrenheit_to_kelvin_dbl(double);

// Poweroff utility
static void on_poweroff();
static void reset_settings_to_default();

// Indication
static void fio_led_on();
static void fio_led_off();
static void fio_led_blink();
static void fio_led_switch();
static void load_led_on();
static void load_led_off();
static void load_led_blink();
static void load_led_switch();
static void user_action_led_on();
static void user_action_led_off();
static void user_action_led_blink();
static void user_action_led_switch();

// Alloc helpers
static void *jfes_my_malloc(size_t);
static void jfes_my_free(void *);
static size_t jfes_my_malloc_cnt;
static size_t jfes_my_free_cnt;

// Procedures
static void thermometer_proc_start();
static void thermometer_proc_stop();

static void brewery_proc_start();
static void brewery_proc_stop();

static void temp_hold_proc_start();
static void temp_hold_proc_stop();
static void temp_hold_timer_proc_start();
static void temp_hold_timer_proc_stop();

// RTC utilites
static void rtc_reset_time();
static void rtc_set_time(const struct rtc_s *);
static void rtc_get_time(struct rtc_s *const);

// Proc callbacks
static void thermometer_proc_rtc_cbk(const void *, size_t);
static void thermometer_proc_adc_cbk(const void *, size_t);
static void brewery_proc_rtc_cbk(const void *, size_t);
static void brewery_proc_adc_cbk(const void *, size_t);
static void temp_hold_proc_adc_cbk(const void *, size_t);
static void temp_hold_proc_rtc_cbk(const void *, size_t);
static void temp_hold_timer_proc_rtc_cbk(const void *, size_t);
static void rtc_callback(const void *, size_t);

// Proc utilities
static void beep();
static void beep_short();

enum menu_mode_e : uint32_t {
  MENU_MODE_ENTRIES_SURFING = 0u,
  MENU_MODE_VALUES_EDITING,
  MENU_MODE_PROCEDURE_THERMOMETER,
  MENU_MODE_PROCEDURE_BREWERY,
  MENU_MODE_PROCEDURE_TEMP_HOLD,
  MENU_MODE_PROCEDURE_TEMP_HOLD_TIMER
};

static enum menu_mode_e state = MENU_MODE_ENTRIES_SURFING;

static const char *brewery_menu_file_name = "brewery_menu.json";
static const char *brewery_menu_variants_file_name = "brewery_menu_variants.json";

static constexpr const char *application_name = "BrewMaster!";
static constexpr const char *firmware_version = "v0.01";
static constexpr int32_t temperature_holder_time_default = 60;
static constexpr const int32_t mashing_default_temperature_kelvin = 335;
static constexpr const int32_t boiling_default_time_min = 60;
static constexpr const int32_t boiling_default_temperature_kelvin = 373;
static constexpr const int32_t boiling_default_power = 100;
static constexpr const int32_t temperature_holder_max_power_default = 50;
static constexpr const int32_t add_hop_default_time1_min = 5;
static constexpr const int32_t add_hop_default_time2_min = 60;
static constexpr const int32_t add_hop_default_time3_min = 15;
static constexpr const int32_t add_hop_default_time4_min = 25;
static constexpr const int32_t pause_default_temperature_kelvin = 335;
static constexpr const int32_t pause_default_time_min = 35;
static constexpr const char *default_temperature_unit = "Kelvin";
static constexpr const int32_t min_temp_kelvin = 273 + 25;
static constexpr const int32_t max_temp_kelvin = 273 + 110;
static constexpr const int32_t step_temp_kelvin = 1u;
static constexpr const int32_t min_time_min = 0;
static constexpr const int32_t max_time_min = 24 * 60;
static constexpr const int32_t step_time_min = 1;
static constexpr const size_t max_user_recipes = 4u;
static constexpr const int32_t min_power_percent = 0;
static constexpr const int32_t max_power_percent = 100;
static constexpr const int32_t step_power_percent = 5;

static constexpr const double default_proportional_coefficient = 0.5f;
static constexpr const double default_integral_coefficient = 0.002f;

static constexpr const double proportional_coefficient_min_val = 0.0f;
static constexpr const double proportional_coefficient_max_val = 25.0f;
static constexpr const double proportional_coefficient_step_val = 0.05f;

static constexpr const double integral_coefficient_min_val = 0.0f;
static constexpr const double integral_coefficient_max_val = 1.0f;
static constexpr const double integral_coefficient_step_val = 0.001f;

static bool menu_changed, task_running, thermometer_proc_running, brewery_proc_running, temp_hold_proc_running, temp_hold_timer_proc_running;
static jfes_config_t json_config{.jfes_malloc = jfes_my_malloc, .jfes_free = jfes_my_free};

enum brewery_proc_state_e : uint32_t {
  BREWERY_PROC_STATE_UNDEFINED = 0u,
  BREWERY_PROC_STATE_WAITING,
  BREWERY_PROC_STATE_MASHING,
  BREWERY_PROC_STATE_PAUSE1,
  BREWERY_PROC_STATE_PAUSE2,
  BREWERY_PROC_STATE_PAUSE3,
  BREWERY_PROC_STATE_PAUSE4,
  BREWERY_PROC_STATE_PAUSE5,
  BREWERY_PROC_STATE_MASHING_OUT,
  BREWERY_PROC_STATE_BOILING,
  BREWERY_PROC_STATE_END
};

enum temp_hold_proc_regulation_state_e : uint32_t { TEMP_HOLD_PROC_REGULATION_STATE_POWER = 0u, TEMP_HOLD_PROC_REGULATION_STATE_TEMPERATURE };
enum brewery_boiling_state_e : uint32_t { BREWERY_PROC_BOILING_STATE_TEMP_REG = 0u, BREWERY_PROC_BOILING_STATE_POWER_REG };

static enum brewery_boiling_state_e brewery_proc_boiling_state;
static enum temp_hold_proc_regulation_state_e temp_hold_proc_regulation_state = TEMP_HOLD_PROC_REGULATION_STATE_TEMPERATURE;
static enum brewery_proc_state_e brewery_proc_state, brewery_proc_next_state, brewery_proc_prev_state;
static uint32_t beep_cnt = 0u;

// Application entry point
void brewery_app_s::entry(void *args) {
  char **arglist;
  char *cl_buffer_copy;
  size_t json_size;
  int32_t rc;
  xTaskHandle blinker;
  lcd_fb_window_ta lcd_fb_window;

  static sha256::sha256_hash_type menu_hash, menu_variants_hash;

  // struct mallinfo info = mallinfo();
  // brewery_app_s::dbg_printfmt("Heap: free : %i, allocated : %i, arena : %i\r\n", info.fordblks, info.uordblks, info.arena);

  menu_semphr = xSemaphoreCreateRecursiveMutex();
  menu_variants_semphr = xSemaphoreCreateRecursiveMutex();
  thermometer_proc_sem = xSemaphoreCreateRecursiveMutex();
  brewery_proc_sem = xSemaphoreCreateRecursiveMutex();
  temp_hold_proc_sem = xSemaphoreCreateRecursiveMutex();
  temp_hold_timer_proc_sem = xSemaphoreCreateRecursiveMutex();

  beep_short();
  fio_led_on();
  if ((rc = mount_fs()) < 0) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto exit;
  }

  // lfs_remove(&lfs, brewery_menu_file_name);
  // lfs_remove(&lfs, brewery_menu_variants_file_name);

  // Read from file or generate default menus
  brewery_dbg_printfmt("Getting menu from %s ...\r\n", brewery_menu_file_name);
  xSemaphoreTakeRecursive(menu_semphr, portMAX_DELAY);
  if (!(menu = get_json_value(brewery_menu_file_name, generate_default_menu, &menu_hash))) {
    lfs_remove(&lfs, brewery_menu_file_name);
    if (!(menu = get_json_value(brewery_menu_file_name, generate_default_menu, &menu_hash))) {
      brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      xSemaphoreGiveRecursive(menu_semphr);
      goto exit;
    }
  }

  xSemaphoreGiveRecursive(menu_semphr);

  brewery_dbg_printfmt("Getting menu variants from %s ...\r\n", brewery_menu_variants_file_name);
  xSemaphoreTakeRecursive(menu_variants_semphr, portMAX_DELAY);
  if (!(menu_variants = get_json_value(brewery_menu_variants_file_name, generate_default_menu_variants, &menu_variants_hash))) {
    lfs_remove(&lfs, brewery_menu_variants_file_name);
    if (!(menu_variants = get_json_value(brewery_menu_variants_file_name, generate_default_menu_variants, &menu_variants_hash))) {
      brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      xSemaphoreGiveRecursive(menu_variants_semphr);
      goto exit;
    }
  }

  xSemaphoreGiveRecursive(menu_variants_semphr);
  fio_led_off();

  // Init LCD display console
  lcd_console_init();
  lcd_fb_clear(&lcd_fb);
  lcd_clear();

  // Reset entry pointers
  current_menu_entry_ptr = nullptr;
  cursor_entry_ptr = nullptr;
  pending_to_save = nullptr;

  // Init callbacks
  if ((rc = init_button_callbacks()) < 0) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto exit;
  }

  // Print Root menu entry with logo to LCD
  trigger_json_val(&lcd_fb, menu, application_name, LCD_ALIGN_CENTER);
  save_pending_semphr = xSemaphoreCreateBinary();
  xSemaphoreGive(save_pending_semphr);
  menu_changed = false;
  task_running = true;
  // Go to loop, observe for pending file savings, work on button callbacks further

loop:
  while (task_running) {
    BaseType_t rc;
    vTaskDelay(1u);
    if (pending_to_save) {
      if ((rc = xSemaphoreTake(save_pending_semphr, portIO_MAX_DELAY * 1000u)) != pdPASS) {
        continue;
      }

      beep_short();
      fio_led_on();
      if (pending_to_save == menu) {

        xSemaphoreTakeRecursive(menu_semphr, portMAX_DELAY);
        write_value_to_file(brewery_menu_file_name, pending_to_save, nullptr);
        xSemaphoreGiveRecursive(menu_semphr);
      } else if (pending_to_save == menu_variants) {

        xSemaphoreTakeRecursive(menu_variants_semphr, portMAX_DELAY);
        write_value_to_file(brewery_menu_variants_file_name, pending_to_save, nullptr);
        xSemaphoreGiveRecursive(menu_variants_semphr);
      }

      fio_led_off();
      pending_to_save = nullptr;
      menu_changed = false;
      xSemaphoreGive(save_pending_semphr);
    }
  }

  // Exit state -- terminate app
exit:
  vSemaphoreDelete(menu_semphr);
  vSemaphoreDelete(menu_variants_semphr);
  vSemaphoreDelete(save_pending_semphr);
  vSemaphoreDelete(thermometer_proc_sem);
  vSemaphoreDelete(brewery_proc_sem);
  vSemaphoreDelete(temp_hold_proc_sem);
  vSemaphoreDelete(temp_hold_timer_proc_sem);

  lfs_unmount(&lfs);

  if ((rc = ::open(&sys, "rcc/rcc0", 3, 2u)) < 0) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
  }

  if ((rc = ::ioctl(&sys, "rcc/rcc0", rcc_drv_ioctl_cmd_e::RCC_REBOOT, nullptr, 0u)) < 0) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
  }

  if ((rc = ::close(&sys, "rcc/rcc0")) < 0) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
  }

  vTaskDelete(nullptr);
}

char **brewery_app_s::split_string_(char *str, const char *delim) {
  char **res = nullptr, *p;
  int32_t n = 0;

  if (!str || !delim) {
    return nullptr;
  }

  p = std::strtok(str, delim);

  while (p) {
    n++;
    res = static_cast<char **>(std::realloc(res, sizeof(char **) * n));
    res[n - 1u] = p;
    p = std::strtok(nullptr, " ");
  }

  // Terminate list
  res = static_cast<char **>(std::realloc(res, sizeof(char **) * n + 1u));
  res[n] = nullptr;
  return res;
}

int32_t brewery_app_s::brewery_dbg_printfmt(const char *fmt, ...) {
  int32_t rc, usart_fd, strlen;
  static char *temp;
  const struct drv_model_cmn_s *usart;

  std::va_list arg;
  va_start(arg, fmt);
  size_t bufsz = std::vsnprintf(nullptr, 0u, fmt, arg);
  temp = static_cast<char *>(std::calloc(bufsz, sizeof(char)));
  strlen = std::vsprintf(temp, fmt, arg);
  va_end(arg);

  if (!debug_log_enabled) {
    goto exit;
  }

  if (!(usart = sys.drv("usart"))) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if (strlen) {
    if ((usart_fd = ::open(usart, "usart1", 3, 3u)) < 0) {
      brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    if ((rc = ::write(usart, usart_fd, "[brewery] : ", std::strlen("[brewery] : "))) < 0) {
      if ((rc = ::close(usart, usart_fd)) < 0) {
        brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
        goto error;
      }
      brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    if ((rc = ::write(usart, usart_fd, temp, std::strlen(temp))) < 0) {
      if ((rc = ::close(usart, usart_fd)) < 0) {
        brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
        goto error;
      }

      brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    if ((rc = ::close(usart, usart_fd)) < 0) {
      brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }
  }

exit:
  std::free(temp);
  return strlen;
error:

  std::free(temp);
  return -1;
}

int32_t brewery_app_s::dbg_printfmt(const char *fmt, ...) {
  int32_t rc, usart_fd, strlen;
  static char *temp;
  const struct drv_model_cmn_s *usart;

  std::va_list arg;
  va_start(arg, fmt);
  size_t bufsz = std::vsnprintf(nullptr, 0u, fmt, arg);
  temp = static_cast<char *>(std::calloc(bufsz, sizeof(char)));
  strlen = std::vsprintf(temp, fmt, arg);
  va_end(arg);

  if (!(usart = sys.drv("usart"))) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if (strlen) {
    if ((usart_fd = ::open(usart, "usart1", 3, 3u)) < 0) {
      brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    if ((rc = ::write(usart, usart_fd, temp, std::strlen(temp))) < 0) {
      if ((rc = ::close(usart, usart_fd)) < 0) {
        brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
        goto error;
      }

      brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    if ((rc = ::close(usart, usart_fd)) < 0) {
      brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }
  }

  std::free(temp);
  return strlen;
error:

  std::free(temp);
  return -1;
}

int32_t brewery_app_s::brewery_dbg_nprint(void *ptr, size_t n) {
  int32_t rc, usart_fd;
  const struct drv_model_cmn_s *usart = sys.drv("usart");

  if ((usart_fd = ::open(usart, "usart1", 3, 3u)) < 0) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = ::write(usart, usart_fd, "[brewery] : ", std::strlen("[brewery] : "))) < 0) {
    if ((rc = ::close(usart, usart_fd)) < 0) {
      brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = ::write(usart, usart_fd, ptr, n)) < 0) {
    if ((rc = ::close(usart, usart_fd)) < 0) {
      brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = ::close(usart, usart_fd)) < 0) {
    if ((rc = ::close(usart, usart_fd)) < 0) {
      brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return n;
error:
  return -1;
}

static int32_t mount_fs() {
  int32_t rc;

  if ((rc = ::open(&sys, "w25qxx/w25qxx_flash", 3, 2u)) < 0) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = ::ioctl(&sys, "w25qxx/w25qxx_flash", w25qxx_ioctl_cmd_e::W25QXX_INIT, nullptr, 0u)) < 0) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = ::close(&sys, "w25qxx/w25qxx_flash")) < 0) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  lfs_config();
  if ((rc = lfs_mount(&lfs, &lfs_cfg)) < 0) {

    // Format to LittleFS
    if ((rc = lfs_format(&lfs, &lfs_cfg)) < 0) {

      // Format failed -- erase whole chip
      if ((rc = ::open(&sys, "w25qxx/w25qxx_flash", 3, 2u)) < 0) {
        brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
        goto error;
      }

      if ((rc = ::ioctl(&sys, "w25qxx/w25qxx_flash", w25qxx_ioctl_cmd_e::W25QXX_ERASE_BULK, nullptr, 0u)) < 0) {
        brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
        goto error;
      }

      if ((rc = ::close(&sys, "w25qxx/w25qxx_flash")) < 0) {
        brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
        goto error;
      }
    }

    // Mount again
    if ((rc = lfs_mount(&lfs, &lfs_cfg)) < 0) {
      brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }
  }

  return rc;
error:
  return -1;
}

static struct jfes_value *generate_default_menu() {
  // If menu file is empty -- create default menu from JSON
  struct jfes_value *root = jfes_create_object_value(&json_config);

  // Thermometer
  for (struct jfes_value *top = root; const char *key : {"Thermometer"}) {
    jfes_set_object_property(&json_config, top, jfes_create_null_value(&json_config), key, std::strlen(key));
  }

  // Temperature holder
  for (struct jfes_value *top = root; const char *key : {"TempHold"}) {
    jfes_set_object_property(&json_config, top, jfes_create_object_value(&json_config), key, std::strlen(key));
  }

  // Temperature holder Timer
  for (struct jfes_value *top = root; const char *key : {"TempHoldTimer"}) {
    jfes_set_object_property(&json_config, top, jfes_create_object_value(&json_config), key, std::strlen(key));
  }

  // Force heater On/Off
  for (struct jfes_value *top = root; const char *key : {"HeaterOn/Off"}) {
    jfes_set_object_property(&json_config, top, jfes_create_null_value(&json_config), key, std::strlen(key));
  }

  // Create "Recipes" submenu
  for (struct jfes_value *top = root; const char *key : {"Recipes"}) {
    jfes_set_object_property(&json_config, top, jfes_create_object_value(&json_config), key, std::strlen(key));
  }

  // Create "Settings" submenu
  for (struct jfes_value *top = root; const char *key : {"Settings"}) {
    jfes_set_object_property(&json_config, top, jfes_create_object_value(&json_config), key, std::strlen(key));
  }

  // // Power OFF
  // for (struct jfes_value *top = root; const char *key : {"Reboot"}) {
  //   jfes_set_object_property(&json_config, top, jfes_create_null_value(&json_config), key, std::strlen(key));
  // }

  // // Reset to defaults
  // for (struct jfes_value *top = root; const char *key : {"Reset"}) {
  //   jfes_set_object_property(&json_config, top, jfes_create_null_value(&json_config), key, std::strlen(key));
  // }

  // Temperature holder
  for (struct jfes_value *top = jfes_get_child(root, "TempHold", std::strlen("TempHold")); const char *key : {"Temperature", "Power(%)", "Start"}) {
    if (!std::strcmp(key, "Temperature")) {
      jfes_set_object_property(&json_config, top, jfes_create_integer_value(&json_config, pause_default_temperature_kelvin), key, std::strlen(key));
    } else if (!std::strcmp(key, "Power(%)")) {
      jfes_set_object_property(&json_config, top, jfes_create_integer_value(&json_config, temperature_holder_max_power_default), key, std::strlen(key));
    } else if (!std::strcmp(key, "Start")) {
      jfes_set_object_property(&json_config, top, jfes_create_null_value(&json_config), key, std::strlen(key));
    }
  }

  // Temperature holder timer
  for (struct jfes_value *top = jfes_get_child(root, "TempHoldTimer", std::strlen("TempHoldTimer")); const char *key : {"Temperature", "Time(min)", "Power(%)", "Start"}) {
    if (!std::strcmp(key, "Temperature")) {
      jfes_set_object_property(&json_config, top, jfes_create_integer_value(&json_config, pause_default_temperature_kelvin), key, std::strlen(key));
    } else if (!std::strcmp(key, "Time(min)")) {
      jfes_set_object_property(&json_config, top, jfes_create_integer_value(&json_config, temperature_holder_time_default), key, std::strlen(key));
    } else if (!std::strcmp(key, "Power(%)")) {
      jfes_set_object_property(&json_config, top, jfes_create_integer_value(&json_config, temperature_holder_max_power_default), key, std::strlen(key));
    } else if (!std::strcmp(key, "Start")) {
      jfes_set_object_property(&json_config, top, jfes_create_null_value(&json_config), key, std::strlen(key));
    }
  }

  // Create "Settings" submenus
  for (struct jfes_value *top = jfes_get_child(root, "Settings", std::strlen("Settings")); const char *key : {"General", "PI"}) {
    jfes_set_object_property(&json_config, top, jfes_create_object_value(&json_config), key, std::strlen(key));
  }

  // Create submenus for "General"
  for (struct jfes_value *top = jfes_get_child(jfes_get_child(root, "Settings", std::strlen("Settings")), "General", std::strlen("General")); const char *key : {"TempUnit"}) {
    jfes_set_object_property(&json_config, top, jfes_create_string_value(&json_config, default_temperature_unit, std::strlen(default_temperature_unit)), key, std::strlen(key));
  }

  // Create submenus for "PI"
  for (struct jfes_value *top = jfes_get_child(jfes_get_child(root, "Settings", std::strlen("Settings")), "PI", std::strlen("PI")); const char *key : {"P", "I"}) {
    if (!std::strcmp(key, "P")) {
      jfes_set_object_property(&json_config, top, jfes_create_double_value(&json_config, default_proportional_coefficient), key, std::strlen(key));
    } else if (!std::strcmp(key, "I")) {
      jfes_set_object_property(&json_config, top, jfes_create_double_value(&json_config, default_integral_coefficient), key, std::strlen(key));
    }
  }

  // Create "Recipes" submenus
  for (struct jfes_value *top = jfes_get_child(root, "Recipes", std::strlen("Recipes")); const char *key : {"New"}) {
    jfes_set_object_property(&json_config, top, jfes_create_object_value(&json_config), key, std::strlen(key));
  }

  // Create submenus for "New"
  for (struct jfes_value *top = jfes_get_child(jfes_get_child(root, "Recipes", std::strlen("Recipes")), "New", std::strlen("New"));
       const char *key : {"Mashing", "Pause1", "Pause2", "Pause3", "Pause4", "Pause5", "MashOut", "Boiling", "AddHop", "Start", "Save", "Remove"}) {
    jfes_set_object_property(&json_config, top, jfes_create_object_value(&json_config), key, std::strlen(key));
  }

  // Create start/save submenus for "New"
  for (struct jfes_value *top = jfes_get_child(jfes_get_child(root, "Recipes", std::strlen("Recipes")), "New", std::strlen("New")); const char *key : {"Start", "Save", "Remove"}) {
    jfes_set_object_property(&json_config, top, jfes_create_null_value(&json_config), key, std::strlen(key));
  }

  // Create submenus for "Mashing"
  for (struct jfes_value *top = jfes_get_child(jfes_get_child(jfes_get_child(root, "Recipes", std::strlen("Recipes")), "New", std::strlen("New")), "Mashing", std::strlen("Mashing"));
       const char *key : {"Temperature"}) {
    jfes_set_object_property(&json_config, top, jfes_create_integer_value(&json_config, mashing_default_temperature_kelvin), key, std::strlen(key));
  }

  // Create submenus for PauseX, Mashout entries
  for (const char *pause_key : {"Pause1", "Pause2", "Pause3", "Pause4", "Pause5", "MashOut"}) {
    for (struct jfes_value *top = jfes_get_child(jfes_get_child(jfes_get_child(root, "Recipes", std::strlen("Recipes")), "New", std::strlen("New")), pause_key, std::strlen(pause_key));
         const char *key : {"Temperature", "Time(min)"}) {
      if (!std::strcmp(key, "Temperature")) {
        jfes_set_object_property(&json_config, top, jfes_create_integer_value(&json_config, pause_default_temperature_kelvin), key, std::strlen(key));
      } else if (!std::strcmp(key, "Time(min)")) {
        jfes_set_object_property(&json_config, top, jfes_create_integer_value(&json_config, pause_default_time_min), key, std::strlen(key));
      }
    }
  }

  // Create submenus for "Boiling"
  for (struct jfes_value *top = jfes_get_child(jfes_get_child(jfes_get_child(root, "Recipes", std::strlen("Recipes")), "New", std::strlen("New")), "Boiling", std::strlen("Boiling"));
       const char *key : {"Temperature"}) {
    jfes_set_object_property(&json_config, top, jfes_create_integer_value(&json_config, boiling_default_temperature_kelvin), key, std::strlen(key));
  }

  // Create submenus for "Boiling"
  for (struct jfes_value *top = jfes_get_child(jfes_get_child(jfes_get_child(root, "Recipes", std::strlen("Recipes")), "New", std::strlen("New")), "Boiling", std::strlen("Boiling"));
       const char *key : {"Time(min)"}) {
    jfes_set_object_property(&json_config, top, jfes_create_integer_value(&json_config, boiling_default_time_min), key, std::strlen(key));
  }

  // Create submenus for "Boiling"
  for (struct jfes_value *top = jfes_get_child(jfes_get_child(jfes_get_child(root, "Recipes", std::strlen("Recipes")), "New", std::strlen("New")), "Boiling", std::strlen("Boiling"));
       const char *key : {"Power(%)"}) {
    jfes_set_object_property(&json_config, top, jfes_create_integer_value(&json_config, boiling_default_power), key, std::strlen(key));
  }

  // Create submenus for "Add hop"
  for (struct jfes_value *top = jfes_get_child(jfes_get_child(jfes_get_child(root, "Recipes", std::strlen("Recipes")), "New", std::strlen("New")), "AddHop", std::strlen("AddHop"));
       const char *key : {"AddHopTime1(min)", "AddHopTime2(min)", "AddHopTime3(min)", "AddHopTime4(min)"}) {
    if (!std::strcmp(key, "AddHopTime1(min)")) {
      jfes_set_object_property(&json_config, top, jfes_create_integer_value(&json_config, add_hop_default_time1_min), key, std::strlen(key));
    } else if (!std::strcmp(key, "AddHopTime2(min)")) {
      jfes_set_object_property(&json_config, top, jfes_create_integer_value(&json_config, add_hop_default_time2_min), key, std::strlen(key));
    } else if (!std::strcmp(key, "AddHopTime3(min)")) {
      jfes_set_object_property(&json_config, top, jfes_create_integer_value(&json_config, add_hop_default_time3_min), key, std::strlen(key));
    } else if (!std::strcmp(key, "AddHopTime4(min)")) {
      jfes_set_object_property(&json_config, top, jfes_create_integer_value(&json_config, add_hop_default_time4_min), key, std::strlen(key));
    }
  }

  return root;
}

static struct jfes_value *generate_default_menu_variants() {
  struct jfes_value *root = jfes_create_object_value(&json_config);

  // String variants
  for (struct jfes_value *top = root; const char *key : {"TempUnit"}) {
    jfes_set_object_property(&json_config, top, jfes_create_array_value(&json_config), key, std::strlen(key));
  }

  // Numeric objects
  for (struct jfes_value *top = root; const char *key : {"Temperature", "Time(min)", "Power(%)", "AddHopTime1(min)", "AddHopTime2(min)", "AddHopTime3(min)", "AddHopTime4(min)"}) {
    jfes_set_object_property(&json_config, top, jfes_create_object_value(&json_config), key, std::strlen(key));
  }

  // Real objects
  for (struct jfes_value *top = root; const char *key : {"P", "I"}) {
    jfes_set_object_property(&json_config, top, jfes_create_object_value(&json_config), key, std::strlen(key));
  }

  // Real variants
  for (const char *key : {"P", "I"}) {
    for (struct jfes_value *top = jfes_get_child(root, key, std::strlen(key)); const char *subkey : {"min", "max", "step"}) {
      if (!std::strcmp(key, "P")) {
        if (!std::strcmp(subkey, "min")) {
          jfes_set_object_property(&json_config, top, jfes_create_double_value(&json_config, proportional_coefficient_min_val), subkey, std::strlen(subkey));
        } else if (!std::strcmp(subkey, "max")) {
          jfes_set_object_property(&json_config, top, jfes_create_double_value(&json_config, proportional_coefficient_max_val), subkey, std::strlen(subkey));
        } else if (!std::strcmp(subkey, "step")) {
          jfes_set_object_property(&json_config, top, jfes_create_double_value(&json_config, proportional_coefficient_step_val), subkey, std::strlen(subkey));
        }
      } else if (!std::strcmp(key, "I")) {
        if (!std::strcmp(subkey, "min")) {
          jfes_set_object_property(&json_config, top, jfes_create_double_value(&json_config, integral_coefficient_min_val), subkey, std::strlen(subkey));
        } else if (!std::strcmp(subkey, "max")) {
          jfes_set_object_property(&json_config, top, jfes_create_double_value(&json_config, integral_coefficient_max_val), subkey, std::strlen(subkey));
        } else if (!std::strcmp(subkey, "step")) {
          jfes_set_object_property(&json_config, top, jfes_create_double_value(&json_config, integral_coefficient_step_val), subkey, std::strlen(subkey));
        }
      }
    }
  }

  // Create "Temperature units" submenus
  for (struct jfes_value *top = jfes_get_child(root, "TempUnit", std::strlen("TempUnit")); const char *var : {"Celsius", "Fahrenheit", "Kelvin"}) {
    jfes_place_to_array(&json_config, top, jfes_create_string_value(&json_config, var, std::strlen(var)));
  }

  for (const char *key : {"Temperature", "Time(min)", "Power(%)", "AddHopTime1(min)", "AddHopTime2(min)", "AddHopTime3(min)", "AddHopTime4(min)"}) {
    for (struct jfes_value *top = jfes_get_child(root, key, std::strlen(key)); const char *subkey : {"min", "max", "step"}) {
      if (!std::strcmp(key, "Temperature")) {
        if (!std::strcmp(subkey, "min")) {
          jfes_set_object_property(&json_config, top, jfes_create_integer_value(&json_config, min_temp_kelvin), subkey, std::strlen(subkey));
        } else if (!std::strcmp(subkey, "max")) {
          jfes_set_object_property(&json_config, top, jfes_create_integer_value(&json_config, max_temp_kelvin), subkey, std::strlen(subkey));
        } else if (!std::strcmp(subkey, "step")) {
          jfes_set_object_property(&json_config, top, jfes_create_integer_value(&json_config, step_temp_kelvin), subkey, std::strlen(subkey));
        }
      } else if (!std::strcmp(key, "Power(%)")) {
        if (!std::strcmp(subkey, "min")) {
          jfes_set_object_property(&json_config, top, jfes_create_integer_value(&json_config, min_power_percent), subkey, std::strlen(subkey));
        } else if (!std::strcmp(subkey, "max")) {
          jfes_set_object_property(&json_config, top, jfes_create_integer_value(&json_config, max_power_percent), subkey, std::strlen(subkey));
        } else if (!std::strcmp(subkey, "step")) {
          jfes_set_object_property(&json_config, top, jfes_create_integer_value(&json_config, step_power_percent), subkey, std::strlen(subkey));
        }
      } else if (!std::strcmp(key, "Time(min)") || !std::strcmp(key, "AddHopTime1(min)") || !std::strcmp(key, "AddHopTime2(min)") || !std::strcmp(key, "AddHopTime3(min)") ||
                 !std::strcmp(key, "AddHopTime4(min)")) {
        if (!std::strcmp(subkey, "min")) {
          jfes_set_object_property(&json_config, top, jfes_create_integer_value(&json_config, min_time_min), subkey, std::strlen(subkey));
        } else if (!std::strcmp(subkey, "max")) {
          jfes_set_object_property(&json_config, top, jfes_create_integer_value(&json_config, max_time_min), subkey, std::strlen(subkey));
        } else if (!std::strcmp(subkey, "step")) {
          jfes_set_object_property(&json_config, top, jfes_create_integer_value(&json_config, step_time_min), subkey, std::strlen(subkey));
        }
      }
    }
  }

  return root;
}

static struct jfes_value *get_json_value(const char *file_path, struct jfes_value *(*default_generator)(), sha256::sha256_hash_type *hash) {
  int32_t rc;
  struct lfs_info finfo;
  struct jfes_value *res, *generated;
  jfes_status status;

  rc = lfs_stat(&lfs, file_path, &finfo);

  // Rc = 0 and file is not empty
  if (!rc && finfo.size) {
    if (!(res = read_value_from_file(file_path, &finfo, hash))) {
      brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    // If file opening error occured or file is empty
  } else if (rc < 0 || !finfo.size) {
    if (!(generated = default_generator())) {
      brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    // Write it to file
    if (!(res = write_value_to_file(file_path, generated, hash))) {
      jfes_free_value(&json_config, generated);
      brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    rc = lfs_stat(&lfs, file_path, &finfo);
    if (!(res = read_value_from_file(file_path, &finfo, hash))) {
      jfes_free_value(&json_config, generated);
      brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    jfes_free_value(&json_config, generated);
  } else {

    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return res;
error:
  return nullptr;
}

static struct jfes_value *write_value_to_file(const char *file_path, struct jfes_value *value, sha256::sha256_hash_type *hash) {
  static char buffer[3072u];
  size_t buffer_size, bytes_written;
  sha256::sha256_hash_type sha256_hash;
  struct lfs_info finfo;
  jfes_status status;
  int32_t rc;

  buffer_size = sizeof(buffer);
  std::memset(buffer, '\0', buffer_size);
  rc = lfs_stat(&lfs, file_path, &finfo);
  if (jfes_status_is_bad(status = jfes_value_to_string(value, buffer, &buffer_size, 0))) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  // brewery_app_s::brewery_dbg_printfmt("JSON to write : %s\r\n", buffer);

  // Write file
  if ((rc = lfs_file_open(&lfs, &file, file_path, LFS_O_RDWR | LFS_O_CREAT)) < 0) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = lfs_file_rewind(&lfs, &file)) < 0) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = lfs_file_write(&lfs, &file, &buffer, buffer_size)) < 0) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  bytes_written = static_cast<size_t>(rc);

  if ((rc = lfs_file_truncate(&lfs, &file, bytes_written)) < 0) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = lfs_file_sync(&lfs, &file)) < 0) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = lfs_file_close(&lfs, &file)) < 0) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  // remember the storage is not updated until the file is closed successfully
  brewery_app_s::brewery_dbg_printfmt("File \"%s\", size = %lu saved\r\n", file_path, bytes_written);

  if (hash) {
    *hash = sha256_hash;
  }

  return value;
error:
  if (file.flags & LFS_F_OPENED) {
    lfs_file_close(&lfs, &file);
  }

  return nullptr;
}

static struct jfes_value *read_value_from_file(const char *file_path, struct lfs_info *finfo, sha256::sha256_hash_type *hash) {
  int32_t rc;
  static char buffer[3072u];
  size_t buffer_size;
  struct jfes_value *res;
  sha256::sha256_hash_type readed_sha256_hash, serialized_sha256_hash;
  bool compare_res;
  jfes_status status;

  buffer_size = sizeof(buffer);
  std::memset(buffer, '\0', buffer_size);

  res = jfes_create_object_value(&json_config);
  brewery_app_s::brewery_dbg_printfmt("Reading existing menu JSON file \"%s\" with size %lu ...\r\n", file_path, finfo->size);

  // File exists -- read JSON data from file
  if ((rc = lfs_file_open(&lfs, &file, file_path, LFS_O_RDWR)) < 0) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = lfs_file_read(&lfs, &file, &buffer, finfo->size)) < 0) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = lfs_file_close(&lfs, &file))) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  // brewery_app_s::brewery_dbg_printfmt("Readed JSON file \"%s\" : %s\r\n", file_path, buffer);
  readed_sha256_hash = sha256::compute(buffer, finfo->size);

  // Deserialize data and serialize back
  if (jfes_status_is_bad(status = jfes_parse_to_value(&json_config, buffer, finfo->size, res))) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if (hash) {
    *hash = readed_sha256_hash;
  }

  return res;
error:
  jfes_free_value(&json_config, res);

  if (file.flags & LFS_F_OPENED) {
    lfs_file_close(&lfs, &file);
  }

  return nullptr;
}

static void print_sha256(const sha256::sha256_hash_type *hash) {
  brewery_app_s::brewery_dbg_printfmt("SHA256 : [");
  for (uint8_t byte : *hash) {
    brewery_app_s::dbg_printfmt("%#02x,", byte);
  }

  brewery_app_s::dbg_printfmt("]\r\n");
}

// LCD utils
static void lcd_console_init() {
  int32_t rc;
  if ((rc = ::open(&sys, "hd44780/hd44780_lcd", 3, 2u)) < 0) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = ::ioctl(&sys, "hd44780/hd44780_lcd", hd44780_ioctl_cmd_e::HD44780_BL_ENABLE, nullptr, 0u)) < 0) {
    if ((rc = ::close(&sys, "hd44780/hd44780_lcd")) < 0) {
      brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = ::ioctl(&sys, "hd44780/hd44780_lcd", hd44780_ioctl_cmd_e::HD44780_INIT_8BIT, nullptr, 0u)) < 0) {
    if ((rc = ::close(&sys, "hd44780/hd44780_lcd")) < 0) {
      brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = ::close(&sys, "hd44780/hd44780_lcd")) < 0) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }
error:
  return;
}

static void lcd_console_deinit() {
  int32_t rc;
  if ((rc = ::open(&sys, "hd44780/hd44780_lcd", 3, 2u)) < 0) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = ::ioctl(&sys, "hd44780/hd44780_lcd", hd44780_ioctl_cmd_e::HD44780_BL_DISABLE, nullptr, 0u)) < 0) {
    if ((rc = ::close(&sys, "hd44780/hd44780_lcd")) < 0) {
      brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = ::close(&sys, "hd44780/hd44780_lcd")) < 0) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }
error:
  return;
}

static void lcd_clear() {
  int32_t rc;
  if ((rc = ::open(&sys, "hd44780/hd44780_lcd", 3, 2u)) < 0) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = ::ioctl(&sys, "hd44780/hd44780_lcd", hd44780_ioctl_cmd_e::HD44780_CLEAR, nullptr, 0u)) < 0) {
    if ((rc = ::close(&sys, "hd44780/hd44780_lcd")) < 0) {
      brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = ::close(&sys, "hd44780/hd44780_lcd")) < 0) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

error:
  return;
}

static void lcd_clear_line(uint8_t line_num) {
  char line[lcd_fb_window_x_size];
  char(*line_ptr)[lcd_fb_window_x_size] = &line;
  std::memset(line, ' ', sizeof(line));
  lcd_print_line(&line_ptr, line_num);
}

static void lcd_fill_line(uint8_t line_num, char c) {
  char line[lcd_fb_window_x_size];
  char(*line_ptr)[lcd_fb_window_x_size] = &line;
  std::memset(line, c, sizeof(line));
  lcd_print_line(&line_ptr, line_num);
}

static void lcd_print_line(char (**line)[lcd_fb_window_x_size], uint8_t line_number) {
  int32_t rc;
  const struct hd44780_cursor_pos_req_s cp_req = {.line = line_number, .col = 0u};
  const char(&window_line_string_ref)[lcd_fb_window_x_size] = **line;

  if ((rc = ::open(&sys, "hd44780/hd44780_lcd", 3, 2u)) < 0) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = ::ioctl(&sys, "hd44780/hd44780_lcd", hd44780_ioctl_cmd_e::HD44780_SET_CURSOR, &cp_req, sizeof(cp_req))) < 0) {
    if ((rc = ::close(&sys, "hd44780/hd44780_lcd")) < 0) {
      brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = ::write(&sys, "hd44780/hd44780_lcd", window_line_string_ref, lcd_columns_num)) < 0) {
    if ((rc = ::close(&sys, "hd44780/hd44780_lcd")) < 0) {
      brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = ::close(&sys, "hd44780/hd44780_lcd")) < 0) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

error:
  return;
}

// Prints Framebuffer window to LCD
static void lcd_flush_fb_window(lcd_fb_window_ta *fb_window) {
  for (uint8_t i = 0u; i < lcd_fb_window_y_size; i++) {
    if ((*fb_window)[i]) {
      char(**line)[lcd_fb_window_x_size] = &(*fb_window)[i];
      lcd_print_line(line, i);
    }
  }
}

// LCD Framebuffer utils
// Clear whole framebuffer
static void lcd_fb_clear(lcd_fb_ta *fb) {
  for (uint32_t i = 0u; i < lcd_fb_y_size; i++) {
    char *line = (*fb)[i];
    std::memset(line, ' ', lcd_fb_x_size);
  }
}

// Clears line in framebuffer
static void lcd_fb_clear_line(lcd_fb_ta *fb, uint8_t line_num) {
  if (line_num < lcd_fb_y_size) {
    char *line = (*fb)[line_num];
    std::memset(line, ' ', lcd_fb_x_size);
  }
}

// Fills line in framebuffer
static void lcd_fb_fill_line(lcd_fb_ta *fb, uint8_t line_num, char c) {
  if (line_num < lcd_fb_y_size) {
    char *line = (*fb)[line_num];
    std::memset(line, c, lcd_fb_x_size);
  }
}

// Prints line to framebuffer
static void lcd_fb_print_line(lcd_fb_ta *fb, uint8_t line_num, const char (**line)[lcd_fb_x_size], enum lcd_align_e align) {
  if (line_num < lcd_fb_y_size) {
    char *fb_line = (*fb)[line_num];
    const char *line_str = **line;

    switch (align) {

    case LCD_ALIGN_LEFT: {
      std::memcpy(fb_line, line_str, lcd_fb_x_size);
    } break;

    case LCD_ALIGN_CENTER: {
      size_t strlen = std::strlen(line_str);
      size_t margin = lcd_fb_x_size > strlen ? (lcd_fb_x_size - strlen) / 2u : 0u;
      std::memset(fb_line, ' ', margin);
      std::memcpy(fb_line + margin, line_str, strlen);
      std::memset(fb_line + margin + strlen, ' ', margin);
    } break;

    case LCD_ALIGN_RIGHT: {
      size_t strlen = std::strlen(line_str);
      size_t margin = lcd_fb_x_size - strlen;
      std::memset(fb_line, ' ', margin);
      std::memcpy(fb_line + margin, line_str, strlen);
    } break;

    default:
      break;
    }
  }
}

static void lcd_get_window(lcd_fb_ta *fb, lcd_fb_window_ta *window, uint8_t ypos, uint8_t xpos, enum lcd_align_e align) {
  if (ypos <= lcd_fb_y_size - lcd_fb_window_y_size && xpos <= lcd_fb_window_x_size) {
    for (uint32_t i = 0u; i < lcd_fb_window_y_size; i++) {

      switch (align) {

      case LCD_ALIGN_LEFT: {
        // Write pointer to xpos char of framebuffer line
        (*window)[i] = reinterpret_cast<char(*)[lcd_fb_window_x_size]>(((*fb)[i + ypos] + xpos));
      } break;

      case LCD_ALIGN_CENTER: {
        // Write pointer to xpos char of framebuffer line
        (*window)[i] = reinterpret_cast<char(*)[lcd_fb_window_x_size]>(((*fb)[i + ypos] + xpos + (lcd_fb_x_size - lcd_fb_window_x_size) / 2u));
      } break;

      case LCD_ALIGN_RIGHT: {
        // Write pointer to xpos char of framebuffer line
        (*window)[i] = reinterpret_cast<char(*)[lcd_fb_window_x_size]>(((*fb)[i + ypos] + xpos + lcd_fb_window_x_size * ((lcd_fb_x_size / lcd_fb_window_x_size) - 1u)));
      } break;
      }
    }
  } else {
    for (uint32_t i = 0u; i < lcd_fb_window_y_size; i++) {

      // Write pointer to xpos char of framebuffer line
      (*window)[i] = nullptr;
    }
  }
}

static void trigger_json_val(lcd_fb_ta *fb, struct jfes_value *json, const char *title, enum lcd_align_e align) {

  // Print title
  uint32_t current_line = 0u;
  lcd_fb_window_ta fb_window;
  char line[lcd_fb_x_size];
  size_t margin;

  // Print JSON entries
  switch (json->type) {
  case jfes_type_integer: {
  handle_integer_type:
    brewery_app_s::brewery_dbg_printfmt("Triggered integer type, name \"%s\", value = %i\r\n", json->name, json->data.int_val);
    lcd_clear();
    lcd_fb_clear(&lcd_fb);

    std::memset(line, '\0', lcd_fb_x_size);
    std::snprintf(line, lcd_fb_x_size, "{%s}", json->name);
    const char(*line_ptr)[lcd_fb_x_size] = &line;
    lcd_fb_print_line(fb, current_line++, &line_ptr, align);
    std::memset(line, ' ', lcd_fb_x_size);
    lcd_fb_print_line(fb, current_line++, &line_ptr, align);
    current_menu_entry_ptr = cursor_entry_ptr;

    if (!std::strcmp(json->name, "Temperature")) {
      // It needs to be converted
      int32_t val = json->data.int_val;
      struct jfes_value *temp_unit_val =
          jfes_get_child(jfes_get_child(jfes_get_child(menu, "Settings", std::strlen("Settings")), "General", std::strlen("General")), "TempUnit", std::strlen("TempUnit"));

      if (!std::strcmp(temp_unit_val->data.string_val.data, "Celsius")) {

        std::snprintf(line, sizeof(line), "[%i %cC]", kelvin_to_celsius(json->data.int_val), '\xdf');
      } else if (!std::strcmp(temp_unit_val->data.string_val.data, "Fahrenheit")) {

        std::snprintf(line, sizeof(line), "[%i %cF]", kelvin_to_fahrenheit(json->data.int_val), '\xdf');
      } else if (!std::strcmp(temp_unit_val->data.string_val.data, "Kelvin")) {

        std::snprintf(line, sizeof(line), "[%i %cK]", json->data.int_val, '\xdf');
      }
    } else {
      std::snprintf(line, sizeof(line), "[%i]", json->data.int_val);
    }

    line_ptr = &line;
    lcd_fb_print_line(fb, current_line++, &line_ptr, align);

    lcd_get_window(fb, &fb_window, 0u, 0u, align);
    lcd_flush_fb_window(&fb_window);

    state = MENU_MODE_VALUES_EDITING;
  } break;

  case jfes_type_boolean: {
  handle_boolean_type:
    brewery_app_s::brewery_dbg_printfmt("Triggered boolean type, name \"%s\", value = %s\r\n", json->name, json->data.bool_val ? "true" : "false");
    // state = MENU_MODE_VALUES_EDITING;
  } break;

  case jfes_type_double: {
  handle_double_type:
    brewery_app_s::brewery_dbg_printfmt("Triggered double type, name \"%s\", value = %.3lf\r\n", json->name, json->data.double_val);

    lcd_clear();
    lcd_fb_clear(&lcd_fb);

    std::memset(line, '\0', lcd_fb_x_size);
    std::snprintf(line, lcd_fb_x_size, "{%s}", json->name);
    const char(*line_ptr)[lcd_fb_x_size] = &line;
    lcd_fb_print_line(fb, current_line++, &line_ptr, align);
    std::memset(line, ' ', lcd_fb_x_size);
    lcd_fb_print_line(fb, current_line++, &line_ptr, align);
    current_menu_entry_ptr = cursor_entry_ptr;

    std::snprintf(line, sizeof(line), "[%.3lf]", json->data.double_val);
    line_ptr = &line;
    lcd_fb_print_line(fb, current_line++, &line_ptr, align);

    lcd_get_window(fb, &fb_window, 0u, 0u, align);
    lcd_flush_fb_window(&fb_window);

    state = MENU_MODE_VALUES_EDITING;
  } break;

  case jfes_type_array: {
  handle_array_type:
    brewery_app_s::brewery_dbg_printfmt("Triggered array type, name \"%s\", values count : %i\r\n", json->name, json->data.array_val->count);
  } break;

  case jfes_type_string: {
  handle_string_type:
    const struct jfes_value *vars = jfes_get_child(menu_variants, json->name, std::strlen(json->name));
    if (vars) {
      brewery_app_s::brewery_dbg_printfmt("Triggered string type, name \"%s\", value : %s, variants : [", json->name, json->data.string_val.data,
                                          vars->data.array_val->count ? ", variants : [" : "\r\n");
      for (uint32_t i = 0u; i < vars->data.array_val->count; i++) {
        brewery_app_s::dbg_printfmt("%s%s", vars->data.array_val->items[i]->data.string_val.data, i != vars->data.array_val->count - 1u ? ", " : "]\r\n");
      }
    }

    lcd_clear();
    lcd_fb_clear(&lcd_fb);

    std::memset(line, '\0', lcd_fb_x_size);
    std::snprintf(line, lcd_fb_x_size, "{%s}", json->name);
    const char(*line_ptr)[lcd_fb_x_size] = &line;
    lcd_fb_print_line(fb, current_line++, &line_ptr, align);
    std::memset(line, ' ', lcd_fb_x_size);
    lcd_fb_print_line(fb, current_line++, &line_ptr, align);
    current_menu_entry_ptr = cursor_entry_ptr;

    std::snprintf(line, sizeof(line), vars ? "[%s]" : "<%s>", json->data.string_val.data);
    line_ptr = &line;
    lcd_fb_print_line(fb, current_line++, &line_ptr, align);

    lcd_get_window(fb, &fb_window, 0u, 0u, align);
    lcd_flush_fb_window(&fb_window);

    state = MENU_MODE_VALUES_EDITING;
  } break;

  case jfes_type_object: {
  handle_object_type:

    lcd_clear();
    lcd_fb_clear(&lcd_fb);

    std::memset(line, '\0', lcd_fb_x_size);

    if (!std::strncmp(application_name, title, std::strlen(application_name))) {
      std::snprintf(line, lcd_fb_x_size, "<%s>", title);
    } else {
      std::snprintf(line, lcd_fb_x_size, "{%s}", title);
    }

    const char(*line_ptr)[lcd_fb_x_size] = &line;
    lcd_fb_print_line(fb, current_line++, &line_ptr, align);
    std::memset(line, ' ', lcd_fb_x_size);
    lcd_fb_print_line(fb, current_line++, &line_ptr, align);

    for (uint32_t i = 0; i < json->data.object_val->count; i++) {
      const char *item_name = json->data.object_val->items[i]->key.data;
      char item_name_line[lcd_fb_x_size];
      std::memset(item_name_line, ' ', lcd_fb_x_size);
      std::memcpy(item_name_line, item_name, json->data.object_val->items[i]->key.size > lcd_fb_x_size ? lcd_fb_x_size : json->data.object_val->items[i]->key.size);
      const char(*item_name_line_ptr)[lcd_fb_x_size] = &item_name_line;
      lcd_fb_print_line(fb, current_line++, &item_name_line_ptr, align);

      switch (json->data.object_val->items[i]->value->type) {
      case jfes_type_null:
      case jfes_type_object: {
      } break;
      case jfes_type_string:
      case jfes_type_boolean:
      case jfes_type_integer: {
      } break;
      default:
        break;
      }
    }

    // Set local variables -- entry pointer to root, cursor to first item in root
    current_menu_entry_ptr = json;
    cursor_entry_ptr = json->data.object_val->items[0u]->value;

    lcd_get_window(fb, &fb_window, 0u, 0u, align);
    set_cursor_position(align);
    lcd_flush_fb_window(&fb_window);

    state = MENU_MODE_ENTRIES_SURFING;
  } break;

  case jfes_type_null: {
  handle_null_type:
    brewery_app_s::brewery_dbg_printfmt("Triggered NULL type, name \"%s\"\r\n", json->name);

    if (!std::strcmp(json->name, "Ac0_On/Off") || !std::strcmp(json->name, "Ac1_On/Off") || !std::strcmp(json->name, "Ac2_On/Off") || !std::strcmp(json->name, "Ac3_On/Off") ||
        !std::strcmp(json->name, "Relay0_On/Off") || !std::strcmp(json->name, "Relay1_On/Off")) {
      int32_t load_num;
      char load_type[32u];
      if (std::sscanf(json->name, "%5[^ \n0123456789]%i_On/Off", load_type, &load_num) == 2u) {
        if (!std::strncmp(load_type, "Ac", std::strlen("Ac"))) {
          ac_load_switch(load_num);
        } else if (!std::strncmp(load_type, "Relay", std::strlen("Relay"))) {
          relay_load_switch(load_num);
        }
      }
    } else if (!std::strcmp(json->name, "Start")) {
      if (!std::strcmp(json->parent->parent->name, "Recipes")) {

        // Start recipe brewing
        brewery_recipe = json->parent;
        brewery_proc_start();
        state = MENU_MODE_PROCEDURE_BREWERY;
      } else if (!std::strcmp(json->parent->name, "TempHold")) {

        // Start TempHold procedure
        brewery_app_s::brewery_dbg_printfmt("TempHold start!\r\n");
        temp_hold_proc_start();
        state = MENU_MODE_PROCEDURE_TEMP_HOLD;
      } else if (!std::strcmp(json->parent->name, "TempHoldTimer")) {

        // Start TempHold procedure
        brewery_app_s::brewery_dbg_printfmt("TempHoldTimer start!\r\n");
        temp_hold_timer_proc_start();
        state = MENU_MODE_PROCEDURE_TEMP_HOLD_TIMER;
      }

      break;

      // If Save new recipe triggered
    } else if (!std::strcmp(json->name, "Save")) {

      char new_recipe_name[16u];
      struct jfes_value *recipes = json->parent->parent;
      size_t user_recipes_cnt = recipes->data.object_val->count - 1u;

      if (user_recipes_cnt < max_user_recipes) {
        struct jfes_value *new_recipe = jfes_create_object_value(&json_config);

        // Create submenus for new recipe
        for (struct jfes_value *top = new_recipe; const char *key : {"Mashing", "Pause1", "Pause2", "Pause3", "Pause4", "Pause5", "MashOut", "Boiling", "AddHop"}) {
          jfes_set_object_property(&json_config, top, jfes_create_object_value(&json_config), key, std::strlen(key));
        }

        // Create start/save submenus for "New"
        for (struct jfes_value *top = new_recipe; const char *key : {"Start", "Save", "Remove"}) {
          jfes_set_object_property(&json_config, top, jfes_create_null_value(&json_config), key, std::strlen(key));
        }

        // Create submenus for "Mashing"
        for (struct jfes_value *top = jfes_get_child(new_recipe, "Mashing", std::strlen("Mashing")); const char *key : {"Temperature"}) {
          jfes_set_object_property(&json_config, top,
                                   jfes_create_integer_value(&json_config, jfes_get_child(jfes_get_child(json->parent, "Mashing", std::strlen("Mashing")), key, std::strlen(key))->data.int_val), key,
                                   std::strlen(key));
        }

        // Create submenus for "PauseX" entries
        for (const char *pause_key : {"Pause1", "Pause2", "Pause3", "Pause4", "Pause5", "MashOut"}) {
          for (struct jfes_value *top = jfes_get_child(new_recipe, pause_key, std::strlen(pause_key)); const char *key : {"Temperature", "Time(min)"}) {
            jfes_set_object_property(&json_config, top,
                                     jfes_create_integer_value(&json_config, jfes_get_child(jfes_get_child(json->parent, pause_key, std::strlen(pause_key)), key, std::strlen(key))->data.int_val), key,
                                     std::strlen(key));
          }
        }

        // Create submenus for "Boiling"
        for (struct jfes_value *top = jfes_get_child(new_recipe, "Boiling", std::strlen("Boiling")); const char *key : {"Temperature", "Time(min)", "Power(%)"}) {
          jfes_set_object_property(&json_config, top,
                                   jfes_create_integer_value(&json_config, jfes_get_child(jfes_get_child(json->parent, "Boiling", std::strlen("Boiling")), key, std::strlen(key))->data.int_val), key,
                                   std::strlen(key));
        }

        // Create submenus for "Add hop"
        for (struct jfes_value *top = jfes_get_child(new_recipe, "AddHop", std::strlen("AddHop")); const char *key : {"AddHopTime1(min)", "AddHopTime2(min)", "AddHopTime3(min)", "AddHopTime4(min)"}) {
          jfes_set_object_property(&json_config, top,
                                   jfes_create_integer_value(&json_config, jfes_get_child(jfes_get_child(json->parent, "AddHop", std::strlen("AddHop")), key, std::strlen(key))->data.int_val), key,
                                   std::strlen(key));
        }

      set_new_recipe_name:
        std::sprintf(new_recipe_name, "user_recipe#%lu", user_recipes_cnt++);
        bool name_exist = false;

        // Check if this name exist
        for (uint32_t j = 0u; j < json->parent->parent->data.object_val->count; j++) {
          if (!std::strcmp(json->parent->parent->data.object_val->items[j]->value->name, new_recipe_name)) {
            std::memset(new_recipe_name, '\0', sizeof(new_recipe_name));
            goto set_new_recipe_name;
          }
        }

        brewery_app_s::brewery_dbg_printfmt("Save new recipe : %s!\r\n", new_recipe_name);

        jfes_set_object_property(&json_config, json->parent->parent, new_recipe, new_recipe_name, std::strlen(new_recipe_name));
        trigger_json_val(&lcd_fb, jfes_get_child(json->parent->parent, new_recipe_name, std::strlen(new_recipe_name)), new_recipe_name);

        xSemaphoreTake(save_pending_semphr, portIO_MAX_DELAY * 1000u);
        pending_to_save = menu;
        xSemaphoreGive(save_pending_semphr);
        menu_changed = true;
      }
    } else if (!std::strcmp(json->name, "Remove")) {

      struct jfes_value *recipe = json->parent;
      if (std::strcmp(json->parent->name, "New")) {
        struct jfes_value *parent = json->parent->parent;
        brewery_app_s::brewery_dbg_printfmt("Removing \"%s\" recipe!\r\n", json->parent->name);
        jfes_remove_object_property(&json_config, parent, json->parent->name, std::strlen(json->parent->name));
        brewery_app_s::brewery_dbg_printfmt("Parent has count : %i!\r\n", parent->data.object_val->count);
        trigger_json_val(&lcd_fb, parent, parent->name);

        xSemaphoreTake(save_pending_semphr, portIO_MAX_DELAY * 1000u);
        pending_to_save = menu;
        xSemaphoreGive(save_pending_semphr);
        menu_changed = true;
      }
    } else if (!std::strcmp(json->name, "Thermometer")) {

      // Start thermometer procedure
      brewery_app_s::brewery_dbg_printfmt("Thermometer start!\r\n");
      thermometer_proc_start();
      state = MENU_MODE_PROCEDURE_THERMOMETER;
      break;
    } else if (!std::strcmp(json->name, "Reboot")) {

      // Reboot
      brewery_app_s::brewery_dbg_printfmt("Power OFF!\r\n");
      on_poweroff();
      break;
    } else if (!std::strcmp(json->name, "Reset")) {

      // Reset
      brewery_app_s::brewery_dbg_printfmt("Reset to defaults!\r\n");
      reset_settings_to_default();
      break;
    } else if (!std::strcmp(json->name, "HeaterOn/Off")) {

      // Switch heater
      brewery_app_s::brewery_dbg_printfmt("Heater switch!\r\n");
      beep_short();
      ac_load_switch(heater_ac_load_num);
      break;
    }

    state = MENU_MODE_ENTRIES_SURFING;
  } break;

  case jfes_type_undefined:
  default: {
  } break;
  }
}

static void set_cursor_position(enum lcd_align_e align) {
  uint8_t line_num;
  const char *entry_name;
  size_t entry_name_len;

  for (uint32_t i = 0u; i < current_menu_entry_ptr->data.object_val->count; i++) {
    if (current_menu_entry_ptr->data.object_val->items[i]->value == cursor_entry_ptr) {
      entry_name = current_menu_entry_ptr->data.object_val->items[i]->key.data;
      entry_name_len = std::strlen(entry_name);
      line_num = i;
      break;
    }
  }

  char line[lcd_fb_x_size];
  std::memset(line, '\0', sizeof(line));
  std::snprintf(line, entry_name_len + 2u, "\x7e%s", entry_name);
  const char(*line_ptr)[lcd_fb_x_size] = &line;
  lcd_fb_print_line(&lcd_fb, line_num + 2u, &line_ptr, align);
}

static void unset_cursor_position(enum lcd_align_e align) {
  int32_t rc;
  uint8_t line_num;
  const char *entry_name;
  size_t entry_name_len;

  for (uint32_t i = 0u; i < current_menu_entry_ptr->data.object_val->count; i++) {
    if (current_menu_entry_ptr->data.object_val->items[i]->value == cursor_entry_ptr) {
      entry_name = current_menu_entry_ptr->data.object_val->items[i]->key.data;
      entry_name_len = std::strlen(entry_name);
      line_num = i;
      break;
    }
  }

  char line[lcd_fb_x_size];
  std::memset(line, '\0', sizeof(line));
  std::memcpy(line, entry_name, entry_name_len);
  const char(*line_ptr)[lcd_fb_x_size] = &line;
  lcd_fb_print_line(&lcd_fb, line_num + 2u, &line_ptr, align);
}

static int32_t init_button_callbacks() {
  int32_t rc, fd;
  const struct drv_model_cmn_s *buttons;
  struct button_irq_mgm_req_s req;

  // Setup first button (UP)
  if (!(buttons = sys.drv("button"))) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((fd = ::open(&sys, "button/button0", 2u, 3u)) < 0) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  req = {.trigger = button_triggers_e::BUTTON_TRIGGER_FALLING, .callback = up_button_callback};
  if ((rc = ::ioctl(&sys, "button/button0", button_ioctl_cmd_e::BUTTON_IRQ_ENABLE, &req, sizeof(req))) < 0) {
    if ((rc = ::close(&sys, "button/button0")) < 0) {
      brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = ::close(&sys, "button/button0")) < 0) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  // Setup second button (SET)
  if ((fd = ::open(&sys, "button/button1", 2u, 3u)) < 0) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  req = {.trigger = button_triggers_e::BUTTON_TRIGGER_FALLING, .callback = set_button_callback};
  if ((rc = ::ioctl(&sys, "button/button1", button_ioctl_cmd_e::BUTTON_IRQ_ENABLE, &req, sizeof(req))) < 0) {
    if ((rc = ::close(&sys, "button/button1")) < 0) {
      brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = ::close(&sys, "button/button1")) < 0) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  // Setup third button (DOWN)
  if ((fd = ::open(&sys, "button/button2", 2u, 3u)) < 0) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  req = {.trigger = button_triggers_e::BUTTON_TRIGGER_FALLING, .callback = down_button_callback};
  if ((rc = ::ioctl(&sys, "button/button2", button_ioctl_cmd_e::BUTTON_IRQ_ENABLE, &req, sizeof(req))) < 0) {
    if ((rc = ::close(&sys, "button/button2")) < 0) {
      brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = ::close(&sys, "button/button2")) < 0) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return 0;
error:
  return -1;
}

static int32_t deinit_button_callbacks() {
  int32_t rc, fd;
  const struct drv_model_cmn_s *buttons;
  struct button_irq_mgm_req_s req;

  // Setup first button (UP)
  if (!(buttons = sys.drv("button"))) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((fd = ::open(&sys, "button/button0", 2u, 3u)) < 0) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  req = {.trigger = button_triggers_e::BUTTON_TRIGGER_FALLING, .callback = up_button_callback};
  if ((rc = ::ioctl(&sys, "button/button0", button_ioctl_cmd_e::BUTTON_IRQ_DISABLE, &req, sizeof(req))) < 0) {
    if ((rc = ::close(&sys, "button/button0")) < 0) {
      brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = ::close(&sys, "button/button0")) < 0) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  // Setup second button (SET)
  if ((fd = ::open(&sys, "button/button1", 2u, 3u)) < 0) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  req = {.trigger = button_triggers_e::BUTTON_TRIGGER_FALLING, .callback = set_button_callback};
  if ((rc = ::ioctl(&sys, "button/button1", button_ioctl_cmd_e::BUTTON_IRQ_DISABLE, &req, sizeof(req))) < 0) {
    if ((rc = ::close(&sys, "button/button1")) < 0) {
      brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = ::close(&sys, "button/button1")) < 0) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  // Setup third button (DOWN)
  if ((fd = ::open(&sys, "button/button2", 2u, 3u)) < 0) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  req = {.trigger = button_triggers_e::BUTTON_TRIGGER_FALLING, .callback = down_button_callback};
  if ((rc = ::ioctl(&sys, "button/button2", button_ioctl_cmd_e::BUTTON_IRQ_DISABLE, &req, sizeof(req))) < 0) {
    if ((rc = ::close(&sys, "button/button2")) < 0) {
      brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = ::close(&sys, "button/button2")) < 0) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return 0;
error:
  return -1;
}

static void up_button_callback(const void *data, size_t size) {
  int32_t fd, rc;
  enum button_state_e button_state;
  const struct button_event_s *event = reinterpret_cast<const struct button_event_s *>(data);
  const struct button_get_state_req_s button_state_req { .state = &button_state };
  struct button_irq_mgm_req_s req;
  bool long_push;

  uint8_t val = event->pin_val;
  uint8_t btn_n = event->btn_num;

  if ((fd = ::open(&sys, "button/button0", 2u, 3u)) < 0) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  brewery_app_s::brewery_dbg_printfmt("UP button callback! Value : %u, button num : %u\r\n", val, btn_n);

  // Check if button is pressed long time
  for (uint32_t i = 0u; i < 32u; i++) {
    vTaskDelay(25u);
    if ((rc = ::ioctl(&sys, "button/button0", button_ioctl_cmd_e::BUTTON_GET_STATE, &button_state_req, sizeof(button_state_req))) < 0) {
      if ((rc = ::close(&sys, "button/button0")) < 0) {
        brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
        goto error;
      }

      brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    if (button_state != button_state_e::BUTTON_PRESSED) {
      long_push = false;
      goto exit;
    }

    long_push = true;
  }

exit:
  if (!long_push) {
    up_short_push_callback();
  } else {

    up_long_push_callback();
  }

  if ((rc = ::close(&sys, "button/button0")) < 0) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return;
error:
  ::close(&sys, "button/button0");
  return;
}

static void down_button_callback(const void *data, size_t size) {
  int32_t fd, rc;
  enum button_state_e button_state;
  const struct button_event_s *event = reinterpret_cast<const struct button_event_s *>(data);
  const struct button_get_state_req_s button_state_req { .state = &button_state };
  struct button_irq_mgm_req_s req;
  bool long_push;

  uint8_t val = event->pin_val;
  uint8_t btn_n = event->btn_num;

  if ((fd = ::open(&sys, "button/button2", 2u, 3u)) < 0) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  brewery_app_s::brewery_dbg_printfmt("DOWN button callback! Value : %u, button num : %u\r\n", val, btn_n);

  // Check if button is pressed long time
  for (uint32_t i = 0u; i < 32u; i++) {
    vTaskDelay(25u);
    if ((rc = ::ioctl(&sys, "button/button2", button_ioctl_cmd_e::BUTTON_GET_STATE, &button_state_req, sizeof(button_state_req))) < 0) {
      if ((rc = ::close(&sys, "button/button2")) < 0) {
        brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
        goto error;
      }

      brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    if (button_state != button_state_e::BUTTON_PRESSED) {
      long_push = false;
      goto exit;
    }

    long_push = true;
  }

exit:
  if (!long_push) {

    down_short_push_callback();
  } else {

    down_long_push_callback();
  }

  if ((rc = ::close(&sys, "button/button2")) < 0) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return;
error:
  ::close(&sys, "button/button2");
  return;
}

static void set_button_callback(const void *data, size_t size) {
  int32_t fd, rc;
  enum button_state_e button_state;
  const struct button_event_s *event = reinterpret_cast<const struct button_event_s *>(data);
  const struct button_get_state_req_s button_state_req { .state = &button_state };
  struct button_irq_mgm_req_s req;
  bool long_push;

  uint8_t val = event->pin_val;
  uint8_t btn_n = event->btn_num;

  if ((fd = ::open(&sys, "button/button1", 3, 3u)) < 0) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  brewery_app_s::brewery_dbg_printfmt("SET button callback! Value : %u, button num : %u\r\n", val, btn_n);

  // Check if button is pressed long time
  for (uint32_t i = 0u; i < 32u; i++) {
    vTaskDelay(25u);
    if ((rc = ::ioctl(&sys, "button/button1", button_ioctl_cmd_e::BUTTON_GET_STATE, &button_state_req, sizeof(button_state_req))) < 0) {
      if ((rc = ::close(&sys, "button/button1")) < 0) {
        brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
        goto error;
      }

      brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    if (button_state != button_state_e::BUTTON_PRESSED) {
      long_push = false;
      goto exit;
    }

    long_push = true;
  }

exit:
  if (!long_push) {

    set_short_push_callback();
  } else {

    set_long_push_callback();
  }

  if ((rc = ::close(&sys, "button/button1")) < 0) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

error:
  ::close(&sys, "button/button1");
  return;
}

static void down_short_push_callback() {
  uint8_t line_num;
  lcd_fb_window_ta fb_window;

  switch (state) {
  case MENU_MODE_ENTRIES_SURFING: {
    for (uint32_t i = 0u; i < current_menu_entry_ptr->data.object_val->count; i++) {
      if (current_menu_entry_ptr->data.object_val->items[i]->value == cursor_entry_ptr) {

        // If this item isn't last item in current menu entry
        if (i != current_menu_entry_ptr->data.object_val->count - 1u) {

          // Erase current item selection
          xSemaphoreTakeRecursive(menu_semphr, portMAX_DELAY);
          unset_cursor_position();

          // Select next item
          cursor_entry_ptr = current_menu_entry_ptr->data.object_val->items[i + 1u]->value;
          set_cursor_position();
          lcd_get_window(&lcd_fb, &fb_window, i, 0u);
          lcd_flush_fb_window(&fb_window);
          xSemaphoreGiveRecursive(menu_semphr);
          brewery_app_s::brewery_dbg_printfmt("Selected next item : %s\r\n", cursor_entry_ptr->name);
          break;

          // Else going to first element
        } else {
          // Erase current item selection
          xSemaphoreTakeRecursive(menu_semphr, portMAX_DELAY);
          unset_cursor_position();

          // Select next item
          cursor_entry_ptr = current_menu_entry_ptr->data.object_val->items[0u]->value;
          set_cursor_position();
          lcd_get_window(&lcd_fb, &fb_window, 0u, 0u);
          lcd_flush_fb_window(&fb_window);
          xSemaphoreGiveRecursive(menu_semphr);
          brewery_app_s::brewery_dbg_printfmt("Going to first item : %s\r\n", cursor_entry_ptr->name);
          break;
        }
      }
    }
  } break;

  case MENU_MODE_VALUES_EDITING: {
    switch (cursor_entry_ptr->type) {
    case jfes_type_double: {
      brewery_app_s::brewery_dbg_printfmt("Switching entire variants : %s\r\n", cursor_entry_ptr->name);
      if (struct jfes_value *entry = jfes_get_child(cursor_entry_ptr->parent, cursor_entry_ptr->name, std::strlen(cursor_entry_ptr->name))) {
        if (const struct jfes_value *vars = jfes_get_child(menu_variants, cursor_entry_ptr->name, std::strlen(cursor_entry_ptr->name))) {
          const struct jfes_value *min = jfes_get_child(vars, "min", std::strlen("min"));
          const struct jfes_value *max = jfes_get_child(vars, "max", std::strlen("max"));
          const struct jfes_value *step = jfes_get_child(vars, "step", std::strlen("step"));

          if (cursor_entry_ptr->data.double_val > min->data.double_val) {
            xSemaphoreTakeRecursive(menu_semphr, portMAX_DELAY);
            entry->data.double_val = cursor_entry_ptr->data.double_val - step->data.double_val;
            trigger_json_val(&lcd_fb, cursor_entry_ptr, cursor_entry_ptr->name);
            menu_changed = true;
            xSemaphoreGiveRecursive(menu_semphr);
          }
        }
      }
    } break;

    case jfes_type_integer: {
      brewery_app_s::brewery_dbg_printfmt("Switching entire variants : %s\r\n", cursor_entry_ptr->name);
      if (struct jfes_value *entry = jfes_get_child(cursor_entry_ptr->parent, cursor_entry_ptr->name, std::strlen(cursor_entry_ptr->name))) {
        if (const struct jfes_value *vars = jfes_get_child(menu_variants, cursor_entry_ptr->name, std::strlen(cursor_entry_ptr->name))) {
          const struct jfes_value *min = jfes_get_child(vars, "min", std::strlen("min"));
          const struct jfes_value *max = jfes_get_child(vars, "max", std::strlen("max"));
          const struct jfes_value *step = jfes_get_child(vars, "step", std::strlen("step"));

          if (cursor_entry_ptr->data.int_val > min->data.int_val) {
            xSemaphoreTakeRecursive(menu_semphr, portMAX_DELAY);
            entry->data.int_val = cursor_entry_ptr->data.int_val - step->data.int_val;
            trigger_json_val(&lcd_fb, cursor_entry_ptr, cursor_entry_ptr->name);
            menu_changed = true;
            xSemaphoreGiveRecursive(menu_semphr);
          }
        }
      }
    } break;

    case jfes_type_boolean:
    case jfes_type_string:
    case jfes_type_array:
    case jfes_type_object:
    case jfes_type_null:
    case jfes_type_undefined:
    default:
      break;
    }
  } break;

  case MENU_MODE_PROCEDURE_TEMP_HOLD ... MENU_MODE_PROCEDURE_TEMP_HOLD_TIMER: {
    switch (temp_hold_proc_regulation_state) {
    case TEMP_HOLD_PROC_REGULATION_STATE_POWER: {
      if (struct jfes_value *entry = jfes_get_child(
              jfes_get_child(menu, state == MENU_MODE_PROCEDURE_TEMP_HOLD ? "TempHold" : "TempHoldTimer", std::strlen(state == MENU_MODE_PROCEDURE_TEMP_HOLD ? "TempHold" : "TempHoldTimer")),
              "Power(%)", std::strlen("Power(%)"))) {
        if (const struct jfes_value *vars = jfes_get_child(menu_variants, "Power(%)", std::strlen("Power(%)"))) {
          const struct jfes_value *min = jfes_get_child(vars, "min", std::strlen("min"));
          const struct jfes_value *max = jfes_get_child(vars, "max", std::strlen("max"));
          const struct jfes_value *step = jfes_get_child(vars, "step", std::strlen("step"));
          if (entry->data.int_val > min->data.int_val) {
            xSemaphoreTakeRecursive(menu_semphr, portMAX_DELAY);
            entry->data.int_val = entry->data.int_val - step->data.int_val;
            xSemaphoreGiveRecursive(menu_semphr);
          }
        }
      }

    } break;

    case TEMP_HOLD_PROC_REGULATION_STATE_TEMPERATURE: {
      if (struct jfes_value *entry = jfes_get_child(
              jfes_get_child(menu, state == MENU_MODE_PROCEDURE_TEMP_HOLD ? "TempHold" : "TempHoldTimer", std::strlen(state == MENU_MODE_PROCEDURE_TEMP_HOLD ? "TempHold" : "TempHoldTimer")),
              "Temperature", std::strlen("Temperature"))) {
        if (const struct jfes_value *vars = jfes_get_child(menu_variants, "Temperature", std::strlen("Temperature"))) {
          const struct jfes_value *min = jfes_get_child(vars, "min", std::strlen("min"));
          const struct jfes_value *max = jfes_get_child(vars, "max", std::strlen("max"));
          const struct jfes_value *step = jfes_get_child(vars, "step", std::strlen("step"));
          if (entry->data.int_val > min->data.int_val) {
            xSemaphoreTakeRecursive(menu_semphr, portMAX_DELAY);
            entry->data.int_val = entry->data.int_val - step->data.int_val;
            xSemaphoreGiveRecursive(menu_semphr);
          }
        }
      }
    } break;
    }
  } break;

  case MENU_MODE_PROCEDURE_BREWERY: {
    switch (brewery_proc_state) {
    case BREWERY_PROC_STATE_BOILING: {
      switch (brewery_proc_boiling_state) {
      case BREWERY_PROC_BOILING_STATE_POWER_REG: {
        if (struct jfes_value *entry = jfes_get_child(jfes_get_child(brewery_recipe, "Boiling", std::strlen("Boiling")), "Power(%)", std::strlen("Power(%)"))) {
          if (const struct jfes_value *vars = jfes_get_child(menu_variants, "Power(%)", std::strlen("Power(%)"))) {
            const struct jfes_value *min = jfes_get_child(vars, "min", std::strlen("min"));
            const struct jfes_value *max = jfes_get_child(vars, "max", std::strlen("max"));
            const struct jfes_value *step = jfes_get_child(vars, "step", std::strlen("step"));

            if (entry->data.int_val > min->data.int_val) {
              xSemaphoreTakeRecursive(menu_semphr, portMAX_DELAY);
              entry->data.int_val = entry->data.int_val - step->data.int_val;
              xSemaphoreGiveRecursive(menu_semphr);
            }
          }
        }
      } break;

      case BREWERY_PROC_BOILING_STATE_TEMP_REG: {
        if (struct jfes_value *entry = jfes_get_child(jfes_get_child(brewery_recipe, "Boiling", std::strlen("Boiling")), "Temperature", std::strlen("Temperature"))) {
          if (const struct jfes_value *vars = jfes_get_child(menu_variants, "Temperature", std::strlen("Temperature"))) {
            const struct jfes_value *min = jfes_get_child(vars, "min", std::strlen("min"));
            const struct jfes_value *max = jfes_get_child(vars, "max", std::strlen("max"));
            const struct jfes_value *step = jfes_get_child(vars, "step", std::strlen("step"));

            if (entry->data.int_val > min->data.int_val) {
              xSemaphoreTakeRecursive(menu_semphr, portMAX_DELAY);
              entry->data.int_val = entry->data.int_val - step->data.int_val;
              xSemaphoreGiveRecursive(menu_semphr);
            }
          }
        }
      } break;

      default:
        break;
      }
    } break;

    case BREWERY_PROC_STATE_MASHING ... BREWERY_PROC_STATE_MASHING_OUT: {
      const char *proc_state_names[]{"Mashing", "Pause1", "Pause2", "Pause3", "Pause4", "Pause5", "MashOut"};
      const char *proc_state_name = proc_state_names[brewery_proc_state - BREWERY_PROC_STATE_MASHING];

      if (struct jfes_value *entry = jfes_get_child(jfes_get_child(brewery_recipe, proc_state_name, std::strlen(proc_state_name)), "Temperature", std::strlen("Temperature"))) {
        if (const struct jfes_value *vars = jfes_get_child(menu_variants, "Temperature", std::strlen("Temperature"))) {
          const struct jfes_value *min = jfes_get_child(vars, "min", std::strlen("min"));
          const struct jfes_value *max = jfes_get_child(vars, "max", std::strlen("max"));
          const struct jfes_value *step = jfes_get_child(vars, "step", std::strlen("step"));

          if (entry->data.int_val > min->data.int_val) {
            xSemaphoreTakeRecursive(menu_semphr, portMAX_DELAY);
            entry->data.int_val = entry->data.int_val - step->data.int_val;
            xSemaphoreGiveRecursive(menu_semphr);
          }
        }
      }
    } break;

    default:
      break;
    }
  } break;

  case MENU_MODE_PROCEDURE_THERMOMETER:
  default:
    break;
  }
}

static void up_short_push_callback() {
  uint8_t line_num;
  lcd_fb_window_ta fb_window;

  switch (state) {
  case MENU_MODE_ENTRIES_SURFING: {
    for (uint32_t i = 0u; i < current_menu_entry_ptr->data.object_val->count; i++) {
      if (current_menu_entry_ptr->data.object_val->items[i]->value == cursor_entry_ptr) {

        // If this item isn't first item in current menu entry
        if (i != 0u) {
          // Erase current item selection
          xSemaphoreTakeRecursive(menu_semphr, portMAX_DELAY);
          unset_cursor_position();

          // Select prevous item
          cursor_entry_ptr = current_menu_entry_ptr->data.object_val->items[i - 1u]->value;
          set_cursor_position();
          lcd_get_window(&lcd_fb, &fb_window, i ? i - 1u : 0u, 0u);
          lcd_flush_fb_window(&fb_window);

          brewery_app_s::brewery_dbg_printfmt("Selected prev item : %s\r\n", cursor_entry_ptr->name);
          xSemaphoreGiveRecursive(menu_semphr);
          break;

          // Else
        } else {

          // Erase current item selection
          xSemaphoreTakeRecursive(menu_semphr, portMAX_DELAY);
          unset_cursor_position();

          // Select prevous item
          cursor_entry_ptr = current_menu_entry_ptr->data.object_val->items[current_menu_entry_ptr->data.object_val->count - 1u]->value;
          set_cursor_position();
          lcd_get_window(&lcd_fb, &fb_window, current_menu_entry_ptr->data.object_val->count - 2u, 0u);
          lcd_flush_fb_window(&fb_window);
          brewery_app_s::brewery_dbg_printfmt("Going to last item : %s\r\n", cursor_entry_ptr->name);
          xSemaphoreGiveRecursive(menu_semphr);
          break;
        }
      }
    }
  } break;

  case MENU_MODE_VALUES_EDITING: {
    switch (cursor_entry_ptr->type) {
    case jfes_type_double: {
      brewery_app_s::brewery_dbg_printfmt("Switching entire variants : %s\r\n", cursor_entry_ptr->name);
      if (struct jfes_value *entry = jfes_get_child(cursor_entry_ptr->parent, cursor_entry_ptr->name, std::strlen(cursor_entry_ptr->name))) {
        if (const struct jfes_value *vars = jfes_get_child(menu_variants, cursor_entry_ptr->name, std::strlen(cursor_entry_ptr->name))) {
          const struct jfes_value *min = jfes_get_child(vars, "min", std::strlen("min"));
          const struct jfes_value *max = jfes_get_child(vars, "max", std::strlen("max"));
          const struct jfes_value *step = jfes_get_child(vars, "step", std::strlen("step"));

          if (cursor_entry_ptr->data.double_val < max->data.double_val) {
            xSemaphoreTakeRecursive(menu_semphr, portMAX_DELAY);
            entry->data.double_val = cursor_entry_ptr->data.double_val + step->data.double_val;
            trigger_json_val(&lcd_fb, cursor_entry_ptr, cursor_entry_ptr->name);
            menu_changed = true;
            xSemaphoreGiveRecursive(menu_semphr);
          }
        }
      }
    } break;

    case jfes_type_integer: {
      brewery_app_s::brewery_dbg_printfmt("Switching entire variants : %s\r\n", cursor_entry_ptr->name);
      if (struct jfes_value *entry = jfes_get_child(cursor_entry_ptr->parent, cursor_entry_ptr->name, std::strlen(cursor_entry_ptr->name))) {
        if (const struct jfes_value *vars = jfes_get_child(menu_variants, cursor_entry_ptr->name, std::strlen(cursor_entry_ptr->name))) {
          const struct jfes_value *min = jfes_get_child(vars, "min", std::strlen("min"));
          const struct jfes_value *max = jfes_get_child(vars, "max", std::strlen("max"));
          const struct jfes_value *step = jfes_get_child(vars, "step", std::strlen("step"));

          if (cursor_entry_ptr->data.int_val < max->data.int_val) {
            xSemaphoreTakeRecursive(menu_semphr, portMAX_DELAY);
            entry->data.int_val = cursor_entry_ptr->data.int_val + step->data.int_val;
            trigger_json_val(&lcd_fb, cursor_entry_ptr, cursor_entry_ptr->name);
            menu_changed = true;
            xSemaphoreGiveRecursive(menu_semphr);
          }
        }
      }
    } break;

    case jfes_type_boolean:
    case jfes_type_string:
    case jfes_type_array:
    case jfes_type_object:
    case jfes_type_null:
    case jfes_type_undefined:
    default:
      break;
    }
  } break;

  case MENU_MODE_PROCEDURE_TEMP_HOLD ... MENU_MODE_PROCEDURE_TEMP_HOLD_TIMER: {
    switch (temp_hold_proc_regulation_state) {
    case TEMP_HOLD_PROC_REGULATION_STATE_POWER: {
      if (struct jfes_value *entry = jfes_get_child(
              jfes_get_child(menu, state == MENU_MODE_PROCEDURE_TEMP_HOLD ? "TempHold" : "TempHoldTimer", std::strlen(state == MENU_MODE_PROCEDURE_TEMP_HOLD ? "TempHold" : "TempHoldTimer")),
              "Power(%)", std::strlen("Power(%)"))) {
        if (const struct jfes_value *vars = jfes_get_child(menu_variants, "Power(%)", std::strlen("Power(%)"))) {
          const struct jfes_value *min = jfes_get_child(vars, "min", std::strlen("min"));
          const struct jfes_value *max = jfes_get_child(vars, "max", std::strlen("max"));
          const struct jfes_value *step = jfes_get_child(vars, "step", std::strlen("step"));

          if (entry->data.int_val < max->data.int_val) {
            xSemaphoreTakeRecursive(menu_semphr, portMAX_DELAY);
            entry->data.int_val = entry->data.int_val + step->data.int_val;
            xSemaphoreGiveRecursive(menu_semphr);
          }
        }
      }
    } break;
    case TEMP_HOLD_PROC_REGULATION_STATE_TEMPERATURE: {
      if (struct jfes_value *entry = jfes_get_child(
              jfes_get_child(menu, state == MENU_MODE_PROCEDURE_TEMP_HOLD ? "TempHold" : "TempHoldTimer", std::strlen(state == MENU_MODE_PROCEDURE_TEMP_HOLD ? "TempHold" : "TempHoldTimer")),
              "Temperature", std::strlen("Temperature"))) {
        if (const struct jfes_value *vars = jfes_get_child(menu_variants, "Temperature", std::strlen("Temperature"))) {
          const struct jfes_value *min = jfes_get_child(vars, "min", std::strlen("min"));
          const struct jfes_value *max = jfes_get_child(vars, "max", std::strlen("max"));
          const struct jfes_value *step = jfes_get_child(vars, "step", std::strlen("step"));

          if (entry->data.int_val < max->data.int_val) {
            xSemaphoreTakeRecursive(menu_semphr, portMAX_DELAY);
            entry->data.int_val = entry->data.int_val + step->data.int_val;
            xSemaphoreGiveRecursive(menu_semphr);
          }
        }
      }
    } break;
    }
  } break;

  case MENU_MODE_PROCEDURE_BREWERY: {
    switch (brewery_proc_state) {
    case BREWERY_PROC_STATE_BOILING: {
      switch (brewery_proc_boiling_state) {
      case BREWERY_PROC_BOILING_STATE_POWER_REG: {
        if (struct jfes_value *entry = jfes_get_child(jfes_get_child(brewery_recipe, "Boiling", std::strlen("Boiling")), "Power(%)", std::strlen("Power(%)"))) {
          if (const struct jfes_value *vars = jfes_get_child(menu_variants, "Power(%)", std::strlen("Power(%)"))) {
            const struct jfes_value *min = jfes_get_child(vars, "min", std::strlen("min"));
            const struct jfes_value *max = jfes_get_child(vars, "max", std::strlen("max"));
            const struct jfes_value *step = jfes_get_child(vars, "step", std::strlen("step"));

            if (entry->data.int_val < max->data.int_val) {
              xSemaphoreTakeRecursive(menu_semphr, portMAX_DELAY);
              entry->data.int_val = entry->data.int_val + step->data.int_val;
              xSemaphoreGiveRecursive(menu_semphr);
            }
          }
        }
      } break;

      case BREWERY_PROC_BOILING_STATE_TEMP_REG: {
        if (struct jfes_value *entry = jfes_get_child(jfes_get_child(brewery_recipe, "Boiling", std::strlen("Boiling")), "Temperature", std::strlen("Temperature"))) {
          if (const struct jfes_value *vars = jfes_get_child(menu_variants, "Temperature", std::strlen("Temperature"))) {
            const struct jfes_value *min = jfes_get_child(vars, "min", std::strlen("min"));
            const struct jfes_value *max = jfes_get_child(vars, "max", std::strlen("max"));
            const struct jfes_value *step = jfes_get_child(vars, "step", std::strlen("step"));

            if (entry->data.int_val < max->data.int_val) {
              xSemaphoreTakeRecursive(menu_semphr, portMAX_DELAY);
              entry->data.int_val = entry->data.int_val + step->data.int_val;
              xSemaphoreGiveRecursive(menu_semphr);
            }
          }
        }
      } break;
      }
    } break;

    case BREWERY_PROC_STATE_MASHING ... BREWERY_PROC_STATE_MASHING_OUT: {
      const char *proc_state_names[]{"Mashing", "Pause1", "Pause2", "Pause3", "Pause4", "Pause5", "MashOut"};
      const char *proc_state_name = proc_state_names[brewery_proc_state - BREWERY_PROC_STATE_MASHING];

      if (struct jfes_value *entry = jfes_get_child(jfes_get_child(brewery_recipe, proc_state_name, std::strlen(proc_state_name)), "Temperature", std::strlen("Temperature"))) {
        if (const struct jfes_value *vars = jfes_get_child(menu_variants, "Temperature", std::strlen("Temperature"))) {
          const struct jfes_value *min = jfes_get_child(vars, "min", std::strlen("min"));
          const struct jfes_value *max = jfes_get_child(vars, "max", std::strlen("max"));
          const struct jfes_value *step = jfes_get_child(vars, "step", std::strlen("step"));

          if (entry->data.int_val < max->data.int_val) {
            xSemaphoreTakeRecursive(menu_semphr, portMAX_DELAY);
            entry->data.int_val = entry->data.int_val + step->data.int_val;
            xSemaphoreGiveRecursive(menu_semphr);
          }
        }
      }
    } break;

    default:
      break;
    }
  } break;

  case MENU_MODE_PROCEDURE_THERMOMETER:
  default:
    break;
  }
}

static void set_short_push_callback() {
  switch (state) {
  case MENU_MODE_ENTRIES_SURFING: {
    brewery_app_s::brewery_dbg_printfmt("Going to entry : %s\r\n", cursor_entry_ptr->name);
    xSemaphoreTakeRecursive(menu_semphr, portMAX_DELAY);
    trigger_json_val(&lcd_fb, cursor_entry_ptr, cursor_entry_ptr->name);
    xSemaphoreGiveRecursive(menu_semphr);
  } break;

  case MENU_MODE_VALUES_EDITING: {
    switch (cursor_entry_ptr->type) {
    case jfes_type_string: {
      brewery_app_s::brewery_dbg_printfmt("Switching entire variants : %s\r\n", cursor_entry_ptr->name);
      if (struct jfes_value *vars = jfes_get_child(menu_variants, cursor_entry_ptr->name, std::strlen(cursor_entry_ptr->name))) {
        const char *next_var;
        if (vars) {
          for (uint32_t i = 0u; i < vars->data.array_val->count; i++) {
            if (!std::strcmp(vars->data.array_val->items[i]->data.string_val.data, cursor_entry_ptr->data.string_val.data)) {
              brewery_app_s::brewery_dbg_printfmt("Current variant : %s\r\n", cursor_entry_ptr->data.string_val.data);
              (i < (vars->data.array_val->count - 1u)) ? next_var = vars->data.array_val->items[i + 1u]->data.string_val.data : next_var = vars->data.array_val->items[0u]->data.string_val.data;
              brewery_app_s::brewery_dbg_printfmt("Next variant : %s\r\n", next_var);
              struct jfes_value *new_val = jfes_create_string_value(&json_config, next_var, std::strlen(next_var));
              struct jfes_value *old_val = jfes_get_child(cursor_entry_ptr->parent, cursor_entry_ptr->name, std::strlen(cursor_entry_ptr->name));

              jfes_set_object_property(&json_config, cursor_entry_ptr->parent, new_val, cursor_entry_ptr->name, std::strlen(cursor_entry_ptr->name));
              cursor_entry_ptr = jfes_get_child(cursor_entry_ptr->parent, cursor_entry_ptr->name, std::strlen(cursor_entry_ptr->name));
              trigger_json_val(&lcd_fb, cursor_entry_ptr, cursor_entry_ptr->name);
              menu_changed = true;
              jfes_free_value(&json_config, old_val);
              break;
            }
          }
        }
      }
    } break;

      // Don't edit this types on SET button press
    case jfes_type_boolean:
    case jfes_type_double:
    case jfes_type_integer:
    case jfes_type_array:
    case jfes_type_object:
    case jfes_type_null:
    case jfes_type_undefined:
    default:
      break;
    }
  } break;

  case MENU_MODE_PROCEDURE_BREWERY: {
    switch (brewery_proc_state) {
    case BREWERY_PROC_STATE_BOILING: {
      brewery_proc_boiling_state = brewery_proc_boiling_state == BREWERY_PROC_BOILING_STATE_TEMP_REG ? BREWERY_PROC_BOILING_STATE_POWER_REG : BREWERY_PROC_BOILING_STATE_TEMP_REG;
    } break;
    default:
      break;
    }
  }

  case MENU_MODE_PROCEDURE_TEMP_HOLD ... MENU_MODE_PROCEDURE_TEMP_HOLD_TIMER: {
    temp_hold_proc_regulation_state =
        temp_hold_proc_regulation_state == TEMP_HOLD_PROC_REGULATION_STATE_TEMPERATURE ? TEMP_HOLD_PROC_REGULATION_STATE_POWER : TEMP_HOLD_PROC_REGULATION_STATE_TEMPERATURE;
  } break;

  case MENU_MODE_PROCEDURE_THERMOMETER:
  default:
    break;
  }
}

static void up_long_push_callback() {
  int32_t rc, fd;
  enum button_state_e button_state;
  const struct button_get_state_req_s button_state_req { .state = &button_state };

  switch (state) {
  case MENU_MODE_VALUES_EDITING: {
    if ((fd = ::open(&sys, "button/button0", 2u, 3u)) < 0) {
      brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    if ((rc = ::ioctl(&sys, "button/button0", button_ioctl_cmd_e::BUTTON_GET_STATE, &button_state_req, sizeof(button_state_req))) < 0) {
      if ((rc = ::close(&sys, "button/button0")) < 0) {
        brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
        goto error;
      }

      brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    do {
      switch (cursor_entry_ptr->type) {
      case jfes_type_double: {
        brewery_app_s::brewery_dbg_printfmt("Switching entire variants : %s\r\n", cursor_entry_ptr->name);
        if (struct jfes_value *entry = jfes_get_child(cursor_entry_ptr->parent, cursor_entry_ptr->name, std::strlen(cursor_entry_ptr->name))) {
          if (const struct jfes_value *vars = jfes_get_child(menu_variants, cursor_entry_ptr->name, std::strlen(cursor_entry_ptr->name))) {
            const struct jfes_value *min = jfes_get_child(vars, "min", std::strlen("min"));
            const struct jfes_value *max = jfes_get_child(vars, "max", std::strlen("max"));
            const struct jfes_value *step = jfes_get_child(vars, "step", std::strlen("step"));

            if (cursor_entry_ptr->data.double_val < max->data.double_val) {
              xSemaphoreTakeRecursive(menu_semphr, portMAX_DELAY);
              entry->data.double_val = cursor_entry_ptr->data.double_val + step->data.double_val;
              trigger_json_val(&lcd_fb, cursor_entry_ptr, cursor_entry_ptr->name);
              xSemaphoreGiveRecursive(menu_semphr);
            }
          }
        }
      } break;

      case jfes_type_integer: {
        brewery_app_s::brewery_dbg_printfmt("Switching entire variants : %s\r\n", cursor_entry_ptr->name);
        if (struct jfes_value *entry = jfes_get_child(cursor_entry_ptr->parent, cursor_entry_ptr->name, std::strlen(cursor_entry_ptr->name))) {
          if (const struct jfes_value *vars = jfes_get_child(menu_variants, cursor_entry_ptr->name, std::strlen(cursor_entry_ptr->name))) {
            const struct jfes_value *min = jfes_get_child(vars, "min", std::strlen("min"));
            const struct jfes_value *max = jfes_get_child(vars, "max", std::strlen("max"));
            const struct jfes_value *step = jfes_get_child(vars, "step", std::strlen("step"));

            if (cursor_entry_ptr->data.int_val < max->data.int_val) {
              xSemaphoreTakeRecursive(menu_semphr, portMAX_DELAY);
              entry->data.int_val = cursor_entry_ptr->data.int_val + step->data.int_val;
              trigger_json_val(&lcd_fb, cursor_entry_ptr, cursor_entry_ptr->name);
              xSemaphoreGiveRecursive(menu_semphr);
            }
          }
        }
      } break;

      case jfes_type_boolean:
      case jfes_type_string:
      case jfes_type_array:
      case jfes_type_object:
      case jfes_type_null:
      case jfes_type_undefined:
      default:
        break;
      }

      vTaskDelay(50u);
      if ((rc = ::ioctl(&sys, "button/button0", button_ioctl_cmd_e::BUTTON_GET_STATE, &button_state_req, sizeof(button_state_req))) < 0) {
        if ((rc = ::close(&sys, "button/button0")) < 0) {
          brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
          goto error;
        }

        brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
        goto error;
      }

      menu_changed = true;
    } while (button_state == button_state_e::BUTTON_PRESSED);

    if ((rc = ::close(&sys, "button/button0")) < 0) {
      brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }
  } break;

  case MENU_MODE_PROCEDURE_TEMP_HOLD:
  case MENU_MODE_PROCEDURE_TEMP_HOLD_TIMER:
  case MENU_MODE_PROCEDURE_THERMOMETER:
  case MENU_MODE_PROCEDURE_BREWERY:
  case MENU_MODE_ENTRIES_SURFING:
  default:
    break;
  }

  return;
error:
  return;
}

static void down_long_push_callback() {
  int32_t rc, fd;
  enum button_state_e button_state;
  const struct button_get_state_req_s button_state_req { .state = &button_state };

  switch (state) {
  case MENU_MODE_VALUES_EDITING: {
    if ((fd = ::open(&sys, "button/button2", 2u, 3u)) < 0) {
      brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    if ((rc = ::ioctl(&sys, "button/button2", button_ioctl_cmd_e::BUTTON_GET_STATE, &button_state_req, sizeof(button_state_req))) < 0) {
      if ((rc = ::close(&sys, "button/button2")) < 0) {
        brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
        goto error;
      }

      brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    do {
      switch (cursor_entry_ptr->type) {
      case jfes_type_double: {
        brewery_app_s::brewery_dbg_printfmt("Switching entire variants : %s\r\n", cursor_entry_ptr->name);
        if (struct jfes_value *entry = jfes_get_child(cursor_entry_ptr->parent, cursor_entry_ptr->name, std::strlen(cursor_entry_ptr->name))) {
          if (const struct jfes_value *vars = jfes_get_child(menu_variants, cursor_entry_ptr->name, std::strlen(cursor_entry_ptr->name))) {
            const struct jfes_value *min = jfes_get_child(vars, "min", std::strlen("min"));
            const struct jfes_value *max = jfes_get_child(vars, "max", std::strlen("max"));
            const struct jfes_value *step = jfes_get_child(vars, "step", std::strlen("step"));

            if (cursor_entry_ptr->data.double_val > min->data.double_val) {
              xSemaphoreTakeRecursive(menu_semphr, portMAX_DELAY);
              entry->data.double_val = cursor_entry_ptr->data.double_val - step->data.double_val;
              trigger_json_val(&lcd_fb, cursor_entry_ptr, cursor_entry_ptr->name);
              xSemaphoreGiveRecursive(menu_semphr);
            }
          }
        }
      } break;

      case jfes_type_integer: {
        brewery_app_s::brewery_dbg_printfmt("Switching entire variants : %s\r\n", cursor_entry_ptr->name);
        if (struct jfes_value *entry = jfes_get_child(cursor_entry_ptr->parent, cursor_entry_ptr->name, std::strlen(cursor_entry_ptr->name))) {
          if (const struct jfes_value *vars = jfes_get_child(menu_variants, cursor_entry_ptr->name, std::strlen(cursor_entry_ptr->name))) {
            const struct jfes_value *min = jfes_get_child(vars, "min", std::strlen("min"));
            const struct jfes_value *max = jfes_get_child(vars, "max", std::strlen("max"));
            const struct jfes_value *step = jfes_get_child(vars, "step", std::strlen("step"));

            if (cursor_entry_ptr->data.int_val > min->data.int_val) {
              xSemaphoreTakeRecursive(menu_semphr, portMAX_DELAY);
              entry->data.int_val = cursor_entry_ptr->data.int_val - step->data.int_val;
              trigger_json_val(&lcd_fb, cursor_entry_ptr, cursor_entry_ptr->name);
              xSemaphoreGiveRecursive(menu_semphr);
            }
          }
        }
      } break;

      case jfes_type_boolean:
      case jfes_type_string:
      case jfes_type_array:
      case jfes_type_object:
      case jfes_type_null:
      case jfes_type_undefined:
      default:
        break;
      }

      vTaskDelay(50u);
      if ((rc = ::ioctl(&sys, "button/button2", button_ioctl_cmd_e::BUTTON_GET_STATE, &button_state_req, sizeof(button_state_req))) < 0) {
        if ((rc = ::close(&sys, "button/button2")) < 0) {
          brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
          goto error;
        }

        brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
        goto error;
      }

      menu_changed = true;
    } while (button_state == button_state_e::BUTTON_PRESSED);

    if ((rc = ::close(&sys, "button/button2")) < 0) {
      brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }
  } break;

  case MENU_MODE_PROCEDURE_BREWERY: {
    brewery_proc_stop();
    trigger_json_val(&lcd_fb, cursor_entry_ptr->parent, cursor_entry_ptr->parent->name ? cursor_entry_ptr->parent->name : application_name);
    state = MENU_MODE_ENTRIES_SURFING;
  } break;

  case MENU_MODE_PROCEDURE_THERMOMETER: {
    thermometer_proc_stop();
    trigger_json_val(&lcd_fb, cursor_entry_ptr->parent, cursor_entry_ptr->parent->name ? cursor_entry_ptr->parent->name : application_name);
    state = MENU_MODE_ENTRIES_SURFING;
  } break;

  case MENU_MODE_PROCEDURE_TEMP_HOLD ... MENU_MODE_PROCEDURE_TEMP_HOLD_TIMER: {
    state == MENU_MODE_PROCEDURE_TEMP_HOLD ? temp_hold_proc_stop() : temp_hold_timer_proc_stop();
    trigger_json_val(&lcd_fb, cursor_entry_ptr->parent, cursor_entry_ptr->parent->name ? cursor_entry_ptr->parent->name : application_name);
    state = MENU_MODE_ENTRIES_SURFING;
  } break;

  case MENU_MODE_ENTRIES_SURFING:
  default:
    brewery_app_s::brewery_dbg_printfmt("DOWN long push on \"%s\"!\r\n", cursor_entry_ptr->name);
    break;
  }

  return;
error:
  return;
}

static void set_long_push_callback() {
  switch (state) {
  case MENU_MODE_VALUES_EDITING: {
    if (current_menu_entry_ptr->parent) {
      brewery_app_s::brewery_dbg_printfmt("SET long push on \"%s\", going to %s!\r\n", cursor_entry_ptr->name,
                                          current_menu_entry_ptr->parent->name ? current_menu_entry_ptr->parent->name : application_name);
      trigger_json_val(&lcd_fb, current_menu_entry_ptr->parent, current_menu_entry_ptr->parent->name ? current_menu_entry_ptr->parent->name : application_name);

      xSemaphoreTake(save_pending_semphr, portMAX_DELAY);
      if (menu_changed) {
        pending_to_save = menu;
      }
      xSemaphoreGive(save_pending_semphr);
    }
  } break;

  case MENU_MODE_ENTRIES_SURFING: {
    if (current_menu_entry_ptr->parent) {
      brewery_app_s::brewery_dbg_printfmt("SET long push on \"%s\", going to %s!\r\n", cursor_entry_ptr->name,
                                          current_menu_entry_ptr->parent->name ? current_menu_entry_ptr->parent->name : application_name);
      xSemaphoreTakeRecursive(menu_semphr, portMAX_DELAY);
      trigger_json_val(&lcd_fb, current_menu_entry_ptr->parent, current_menu_entry_ptr->parent->name ? current_menu_entry_ptr->parent->name : application_name);
      xSemaphoreGiveRecursive(menu_semphr);
    }
  } break;

  case MENU_MODE_PROCEDURE_BREWERY: {
    switch (brewery_proc_state) {
    case BREWERY_PROC_STATE_WAITING: {
      switch (brewery_proc_prev_state) {
      case BREWERY_PROC_STATE_UNDEFINED:
      case BREWERY_PROC_STATE_MASHING:
      case BREWERY_PROC_STATE_PAUSE1 ... BREWERY_PROC_STATE_PAUSE5:
      case BREWERY_PROC_STATE_MASHING_OUT:
      case BREWERY_PROC_STATE_BOILING: {

        user_action_led_off();
        brewery_proc_state = brewery_proc_next_state;
        rtc_reset_time();
        beep_cnt = 0u;
      } break;
      default:
        break;
      }
    } break;

    default:
      break;
    }
  } break;

  case MENU_MODE_PROCEDURE_THERMOMETER:
  case MENU_MODE_PROCEDURE_TEMP_HOLD:
  case MENU_MODE_PROCEDURE_TEMP_HOLD_TIMER:
  default:
    break;
  }
}

static void ac_load_on(int32_t n) {
  int32_t rc, fd;
  char path[32u];
  enum ac_load_state_e state;
  std::snprintf(path, sizeof(path), "ac_load/acload%i", n);
  struct ac_load_get_state_req_s req {
    .state = &state
  };

  if ((fd = ::open(&sys, path, 3, 3u)) < 0) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = ::ioctl(&sys, path, ac_load_ioctl_cmd_e::AC_LOAD_ON, nullptr, 0u)) < 0) {
    if ((rc = ::close(&sys, path)) < 0) {
      brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  load_led_on();
  if ((rc = ::close(&sys, path)) < 0) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return;
error:
  return;
}

static void ac_load_off(int32_t n) {
  int32_t rc, fd;
  char path[32u];
  enum ac_load_state_e state;
  std::snprintf(path, sizeof(path), "ac_load/acload%i", n);
  struct ac_load_get_state_req_s req {
    .state = &state
  };

  if ((fd = ::open(&sys, path, 3, 3u)) < 0) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = ::ioctl(&sys, path, ac_load_ioctl_cmd_e::AC_LOAD_OFF, nullptr, 0u)) < 0) {
    if ((rc = ::close(&sys, path)) < 0) {
      brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  load_led_off();
  if ((rc = ::close(&sys, path)) < 0) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return;
error:
  return;
}

static void ac_load_switch(int32_t n) {
  int32_t rc, fd;
  char path[32u];
  enum ac_load_state_e state;
  std::snprintf(path, sizeof(path), "ac_load/acload%i", n);
  struct ac_load_get_state_req_s req {
    .state = &state
  };

  if ((fd = ::open(&sys, path, 3, 3u)) < 0) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = ::ioctl(&sys, path, ac_load_ioctl_cmd_e::AC_LOAD_GET_STATE, &req, sizeof(req))) < 0) {
    if ((rc = ::close(&sys, path)) < 0) {
      brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if (state == ac_load_state_e::AC_LOAD_STATE_ON) {
    ac_load_off(n);
  } else if (state == ac_load_state_e::AC_LOAD_STATE_OFF) {
    ac_load_on(n);
  }

  if ((rc = ::close(&sys, path)) < 0) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return;
error:
  return;
}

static void relay_load_on(int32_t n) {
  int32_t rc, fd;
  char path[32u];
  enum relay_load_state_e state;
  std::snprintf(path, sizeof(path), "relay_load/relayload%i", n);
  struct relay_load_get_state_req_s req {
    .state = &state
  };

  if ((fd = ::open(&sys, path, 3, 3u)) < 0) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = ::ioctl(&sys, path, relay_load_ioctl_cmd_e::RELAY_LOAD_ON, nullptr, 0u)) < 0) {
    if ((rc = ::close(&sys, path)) < 0) {
      brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = ::close(&sys, path)) < 0) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return;
error:
  return;
}

static void relay_load_off(int32_t n) {
  int32_t rc, fd;
  char path[32u];
  enum relay_load_state_e state;
  std::snprintf(path, sizeof(path), "relay_load/relayload%i", n);
  struct relay_load_get_state_req_s req {
    .state = &state
  };

  if ((fd = ::open(&sys, path, 3, 3u)) < 0) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = ::ioctl(&sys, path, relay_load_ioctl_cmd_e::RELAY_LOAD_OFF, nullptr, 0u)) < 0) {
    if ((rc = ::close(&sys, path)) < 0) {
      brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = ::close(&sys, path)) < 0) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return;
error:
  return;
}

static void relay_load_switch(int32_t n) {
  int32_t rc, fd;
  char path[32u];
  enum relay_load_state_e state;
  std::snprintf(path, sizeof(path), "relay_load/relayload%i", n);
  struct relay_load_get_state_req_s req {
    .state = &state
  };

  if ((fd = ::open(&sys, path, 3, 3u)) < 0) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = ::ioctl(&sys, path, relay_load_ioctl_cmd_e::RELAY_LOAD_GET_STATE, &req, sizeof(req))) < 0) {
    if ((rc = ::close(&sys, path)) < 0) {
      brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if (state == relay_load_state_e::RELAY_LOAD_STATE_ON) {
    relay_load_off(n);
  } else if (state == relay_load_state_e::RELAY_LOAD_STATE_OFF) {
    relay_load_on(n);
  }

  if ((rc = ::close(&sys, path)) < 0) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

error:
  return;
}

// Temperature conversion utilites
static int32_t celsius_to_kelvin(int32_t celsius) { return celsius + 273u; }

static int32_t celsius_to_fahrenheit(int32_t celsius) {
  double celsius_dbl = celsius;
  int32_t res = (((celsius_dbl * 9.0f) / 5.0f) + 32.0f);
  return res;
}

static int32_t kelvin_to_celsius(int32_t kelvin) { return kelvin - 273u; }

static int32_t kelvin_to_fahrenheit(int32_t kelvin) {
  double celsius_dbl = kelvin_to_celsius(kelvin);
  int32_t res = (((celsius_dbl * 9.0f) / 5.0f) + 32.0f);
  return res;
}

static int32_t fahrenheit_to_celsius(int32_t fahrenheit) {
  double fahrenheit_dbl = fahrenheit;
  int32_t res = ((fahrenheit_dbl - 32.0f) * 5.0f) / 9.0f;
  return res;
}

static int32_t fahrenheit_to_kelvin(int32_t fahrenheit) { return celsius_to_kelvin(fahrenheit_to_celsius(fahrenheit)); }

// Temperature conversion utilites (double format)
static double celsius_to_kelvin_dbl(double celsius) { return celsius + 273.0f; }
static double celsius_to_fahrenheit_dbl(double celsius) { return (((celsius * 9.0f) / 5.0f) + 32.0f); }
static double kelvin_to_celsius_dbl(double kelvin) { return kelvin - 273.0f; }
static double kelvin_to_fahrenheit_dbl(double kelvin) { return (((kelvin_to_celsius(kelvin) * 9.0f) / 5.0f) + 32.0f); }
static double fahrenheit_to_celsius_dbl(double fahrenheit) { return ((fahrenheit - 32.0f) * 5.0f) / 9.0f; }
static double fahrenheit_to_kelvin_dbl(double fahrenheit) { return celsius_to_kelvin(fahrenheit_to_celsius(fahrenheit)); }

static void on_poweroff() {
  lcd_clear();
  lcd_console_deinit();
  task_running = false;
}

static void reset_settings_to_default() {
  int32_t rc;
  sha256::sha256_hash_type menu_hash, menu_variants_hash;

  lfs_remove(&lfs, brewery_menu_file_name);
  lfs_remove(&lfs, brewery_menu_variants_file_name);

  jfes_free_value(&json_config, menu);
  jfes_free_value(&json_config, menu_variants);
  beep_short();
  fio_led_on();

  // Read from file or generate default menus
  brewery_app_s::brewery_dbg_printfmt("Getting menu from %s ...\r\n", brewery_menu_file_name);
  xSemaphoreTakeRecursive(menu_semphr, portMAX_DELAY);
  if (!(menu = get_json_value(brewery_menu_file_name, generate_default_menu, &menu_hash))) {
    if (!(menu = get_json_value(brewery_menu_file_name, generate_default_menu, &menu_hash))) {
      brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      xSemaphoreGiveRecursive(menu_semphr);
      goto exit;
    }
  }

  xSemaphoreGiveRecursive(menu_semphr);

  xSemaphoreTakeRecursive(menu_variants_semphr, portMAX_DELAY);
  if (!(menu_variants = get_json_value(brewery_menu_variants_file_name, generate_default_menu_variants, &menu_variants_hash))) {
    if (!(menu_variants = get_json_value(brewery_menu_variants_file_name, generate_default_menu_variants, &menu_variants_hash))) {
      brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      xSemaphoreGiveRecursive(menu_variants_semphr);
      goto exit;
    }
  }

  xSemaphoreGiveRecursive(menu_variants_semphr);
  fio_led_off();

  // Init LCD display console
  lcd_fb_clear(&lcd_fb);
  lcd_clear();

  // Reset entry pointers
  current_menu_entry_ptr = nullptr;
  cursor_entry_ptr = nullptr;
  pending_to_save = nullptr;

  // Print Root menu entry with logo to LCD
  trigger_json_val(&lcd_fb, menu, application_name, LCD_ALIGN_CENTER);
  menu_changed = false;
  task_running = true;

exit:
  return;
}

static void fio_led_on() {
  int32_t rc, fd;
  uint8_t val = 1u;
  if ((fd = ::open(&sys, "leds/led0", 3, 3u)) < 0) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto end;
  }

  if ((rc = ::write(&sys, "leds/led0", &val, sizeof(val))) < 0) {
    if ((rc = ::close(&sys, "leds/led0")) < 0) {
      brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto end;
    }

    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto end;
  }

  if ((rc = ::close(&sys, "leds/led0")) < 0) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto end;
  }

end:
  return;
}

static void fio_led_off() {
  int32_t rc, fd;
  uint8_t val = 0u;
  if ((fd = ::open(&sys, "leds/led0", 3, 3u)) < 0) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto end;
  }

  if ((rc = ::write(&sys, "leds/led0", &val, sizeof(val))) < 0) {
    if ((rc = ::close(&sys, "leds/led0")) < 0) {
      brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto end;
    }

    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto end;
  }

  if ((rc = ::close(&sys, "leds/led0")) < 0) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto end;
  }

end:
  return;
}

static void fio_led_blink() {
  int32_t rc, fd;
  uint8_t val = 0u;

  struct led_blink_req_s led_req {
    .up_ticks = 50u, .down_ticks = 50u, .n_times = 5, .delay_fn = vTaskDelay,
  };

  if ((fd = ::open(&sys, "leds/led0", 3, 3u)) < 0) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto end;
  }

  if ((rc = ::ioctl(&sys, "leds/led0", LEDS_BLINK, &led_req, sizeof(led_req))) < 0) {
    if ((rc = ::close(&sys, "leds/led0")) < 0) {
      brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto end;
    }

    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto end;
  }

  if ((rc = ::close(&sys, "leds/led0")) < 0) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto end;
  }

end:
  return;
}

static void fio_led_switch() {
  int32_t rc, fd;
  uint16_t val;

  struct led_blink_req_s led_req {
    .up_ticks = 50u, .down_ticks = 50u, .n_times = 5, .delay_fn = vTaskDelay,
  };

  if ((fd = ::open(&sys, "leds/led0", 3, 3u)) < 0) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto end;
  }

  if ((rc = ::read(&sys, "leds/led0", &val, sizeof(val))) < 0) {
    if ((rc = ::close(&sys, "leds/led0")) < 0) {
      brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto end;
    }

    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto end;
  }

  val ? fio_led_off() : fio_led_on();

  if ((rc = ::close(&sys, "leds/led0")) < 0) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto end;
  }

end:
  return;
}

static void load_led_on() {
  int32_t rc, fd;
  uint8_t val = 1u;
  if ((fd = ::open(&sys, "leds/led1", 3, 3u)) < 0) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto end;
  }

  if ((rc = ::write(&sys, "leds/led1", &val, sizeof(val))) < 0) {
    if ((rc = ::close(&sys, "leds/led1")) < 0) {
      brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto end;
    }

    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto end;
  }

  if ((rc = ::close(&sys, "leds/led1")) < 0) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto end;
  }

end:
  return;
}

static void load_led_off() {
  int32_t rc, fd;
  uint8_t val = 0u;
  if ((fd = ::open(&sys, "leds/led1", 3, 3u)) < 0) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto end;
  }

  if ((rc = ::write(&sys, "leds/led1", &val, sizeof(val))) < 0) {
    if ((rc = ::close(&sys, "leds/led1")) < 0) {
      brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto end;
    }

    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto end;
  }

  if ((rc = ::close(&sys, "leds/led1")) < 0) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto end;
  }

end:
  return;
}

static void load_led_blink() {
  int32_t rc, fd;
  uint8_t val = 0u;

  struct led_blink_req_s led_req {
    .up_ticks = 50u, .down_ticks = 50u, .n_times = 5, .delay_fn = vTaskDelay,
  };

  if ((fd = ::open(&sys, "leds/led1", 3, 3u)) < 0) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto end;
  }

  if ((rc = ::ioctl(&sys, "leds/led1", LEDS_BLINK, &led_req, sizeof(led_req))) < 0) {
    if ((rc = ::close(&sys, "leds/led1")) < 0) {
      brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto end;
    }

    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto end;
  }

  if ((rc = ::close(&sys, "leds/led1")) < 0) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto end;
  }

end:
  return;
}

static void load_led_switch() {
  int32_t rc, fd;
  uint16_t val;

  struct led_blink_req_s led_req {
    .up_ticks = 50u, .down_ticks = 50u, .n_times = 5, .delay_fn = vTaskDelay,
  };

  if ((fd = ::open(&sys, "leds/led1", 3, 3u)) < 0) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto end;
  }

  if ((rc = ::read(&sys, "leds/led1", &val, sizeof(val))) < 0) {
    if ((rc = ::close(&sys, "leds/led1")) < 0) {
      brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto end;
    }

    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto end;
  }

  val ? load_led_off() : load_led_on();

  if ((rc = ::close(&sys, "leds/led1")) < 0) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto end;
  }

end:
  return;
}

static void user_action_led_on() {
  int32_t rc, fd;
  uint8_t val = 1u;
  if ((fd = ::open(&sys, "leds/led2", 3, 3u)) < 0) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto end;
  }

  if ((rc = ::write(&sys, "leds/led2", &val, sizeof(val))) < 0) {
    if ((rc = ::close(&sys, "leds/led2")) < 0) {
      brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto end;
    }

    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto end;
  }

  if ((rc = ::close(&sys, "leds/led2")) < 0) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto end;
  }

end:
  return;
}

static void user_action_led_off() {
  int32_t rc, fd;
  uint8_t val = 0u;
  if ((fd = ::open(&sys, "leds/led2", 3, 3u)) < 0) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto end;
  }

  if ((rc = ::write(&sys, "leds/led2", &val, sizeof(val))) < 0) {
    if ((rc = ::close(&sys, "leds/led2")) < 0) {
      brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto end;
    }

    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto end;
  }

  if ((rc = ::close(&sys, "leds/led2")) < 0) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto end;
  }

end:
  return;
}

static void user_action_led_blink() {
  int32_t rc, fd;
  uint8_t val = 0u;

  struct led_blink_req_s led_req {
    .up_ticks = 50u, .down_ticks = 50u, .n_times = 5, .delay_fn = vTaskDelay,
  };

  if ((fd = ::open(&sys, "leds/led2", 3, 3u)) < 0) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto end;
  }

  if ((rc = ::ioctl(&sys, "leds/led2", LEDS_BLINK, &led_req, sizeof(led_req))) < 0) {
    if ((rc = ::close(&sys, "leds/led2")) < 0) {
      brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto end;
    }

    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto end;
  }

  if ((rc = ::close(&sys, "leds/led2")) < 0) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto end;
  }

end:
  return;
}

static void user_action_led_switch() {
  int32_t rc, fd;
  uint16_t val;

  struct led_blink_req_s led_req {
    .up_ticks = 50u, .down_ticks = 50u, .n_times = 5, .delay_fn = vTaskDelay,
  };

  if ((fd = ::open(&sys, "leds/led2", 3, 3u)) < 0) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto end;
  }

  if ((rc = ::read(&sys, "leds/led2", &val, sizeof(val))) < 0) {
    if ((rc = ::close(&sys, "leds/led2")) < 0) {
      brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto end;
    }

    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto end;
  }

  val ? user_action_led_off() : user_action_led_on();

  if ((rc = ::close(&sys, "leds/led2")) < 0) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto end;
  }

end:
  return;
}

static void *jfes_my_malloc(size_t size) { return std::malloc(size); }

static void jfes_my_free(void *ptr) { std::free(ptr); }

static void thermometer_proc_start() {
  int32_t rc, rtc_fd, adc_fd;

  struct rtc_s rtc_time {
    .hour = 0u, .min = 0u, .sec = 0u
  };

  struct rtc_set_time_req_s rtc_time_req {
    .rtc = &rtc_time
  };

  struct rtc_callback_req_s rtc_cbk_req {
    .callback = rtc_callback
  };

  struct rtc_irq_req_s rtc_irq_req {
    .priority = 5u
  };

  if ((rtc_fd = ::open(&sys, "rtc/rtc0", 3, 3u)) < 0) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto exit;
  }

  if ((rc = ::ioctl(&sys, "rtc/rtc0", rtc_ioctl_cmd_e::RTC_SET_TIME, &rtc_time_req, sizeof(rtc_time_req))) < 0) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto exit;
  }

  if ((rc = ::ioctl(&sys, "rtc/rtc0", rtc_ioctl_cmd_e::RTC_ON_SECOND_SET, &rtc_cbk_req, sizeof(rtc_cbk_req))) < 0) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto exit;
  }

  if ((rc = ::ioctl(&sys, "rtc/rtc0", rtc_ioctl_cmd_e::RTC_IRQ_ENABLE, &rtc_irq_req, sizeof(rtc_irq_req))) < 0) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto exit;
  }

  if ((adc_fd = ::open(&sys, "ads1118/ads1118_adc", 3, 3u)) < 0) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto exit;
  }

exit:
  if ((rc = ::close(&sys, "ads1118/ads1118_adc")) < 0) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
  }

  if ((rc = ::close(&sys, "rtc/rtc0")) < 0) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
  }

  beep_short();
  thermometer_proc_running = true;
  return;
}

static void thermometer_proc_stop() {
  int32_t rc, rtc_fd, adc_fd;
  xSemaphoreTakeRecursive(thermometer_proc_sem, portMAX_DELAY);
  // if ((rtc_fd = ::open(&sys, "rtc/rtc0", 3, 3u)) < 0) {
  //   brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
  //   goto exit;
  // }

  // if ((rc = ::ioctl(&sys, "rtc/rtc0", rtc_ioctl_cmd_e::RTC_IRQ_DISABLE, nullptr, 0u)) < 0) {
  //   brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
  //   goto exit;
  // }

  // if ((rc = ::ioctl(&sys, "rtc/rtc0", rtc_ioctl_cmd_e::RTC_ON_SECOND_RESET, nullptr, 0u)) < 0) {
  //   brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
  //   goto exit;
  // }

exit:
  // if ((rc = ::close(&sys, "rtc/rtc0")) < 0) {
  //   brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
  // }

  beep_short();
  thermometer_proc_running = false;
  xSemaphoreGiveRecursive(thermometer_proc_sem);
}

static void brewery_proc_start() {
  int32_t rc, rtc_fd, adc_fd;

  struct rtc_s rtc_time {
    .hour = 0u, .min = 0u, .sec = 0u
  };

  struct rtc_set_time_req_s rtc_time_req {
    .rtc = &rtc_time
  };

  struct rtc_callback_req_s rtc_cbk_req {
    .callback = rtc_callback
  };

  struct rtc_irq_req_s rtc_irq_req {
    .priority = 5u
  };

  if ((rtc_fd = ::open(&sys, "rtc/rtc0", 3, 3u)) < 0) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto exit;
  }

  if ((rc = ::ioctl(&sys, "rtc/rtc0", rtc_ioctl_cmd_e::RTC_SET_TIME, &rtc_time_req, sizeof(rtc_time_req))) < 0) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto exit;
  }

  if ((rc = ::ioctl(&sys, "rtc/rtc0", rtc_ioctl_cmd_e::RTC_ON_SECOND_SET, &rtc_cbk_req, sizeof(rtc_cbk_req))) < 0) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto exit;
  }

  if ((rc = ::ioctl(&sys, "rtc/rtc0", rtc_ioctl_cmd_e::RTC_IRQ_ENABLE, &rtc_irq_req, sizeof(rtc_irq_req))) < 0) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto exit;
  }

  if ((adc_fd = ::open(&sys, "ads1118/ads1118_adc", 3, 3u)) < 0) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto exit;
  }

exit:
  if ((rc = ::close(&sys, "ads1118/ads1118_adc")) < 0) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
  }

  if ((rc = ::close(&sys, "rtc/rtc0")) < 0) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
  }

  brewery_proc_prev_state = BREWERY_PROC_STATE_UNDEFINED;
  brewery_proc_state = BREWERY_PROC_STATE_WAITING;
  brewery_proc_next_state = BREWERY_PROC_STATE_MASHING;
  brewery_proc_running = true;
  return;
}

static void brewery_proc_stop() {
  int32_t rc, rtc_fd, adc_fd;
  xSemaphoreTakeRecursive(brewery_proc_sem, portMAX_DELAY);
exit:

  beep_short();
  ac_load_off(heater_ac_load_num);
  user_action_led_off();
  brewery_proc_running = false;
  xSemaphoreGiveRecursive(brewery_proc_sem);
}

static void thermometer_proc_rtc_cbk(const void *data, size_t size) {
  const size_t seconds = *static_cast<const size_t *>(data);
  int32_t rc, adc_fd, rtc_fd;
  uint16_t adc_val;
  double temperature;
  char line[lcd_fb_x_size];
  size_t current_line = 0u, margin = 0u;
  const char(*line_ptr)[lcd_fb_x_size];
  lcd_fb_window_ta window;
  struct rtc_s rtc;

  const struct rtc_get_time_req_s rtc_req { .rtc = &rtc };

  const char *temp_unit =
      jfes_get_child(jfes_get_child(jfes_get_child(menu, "Settings", std::strlen("Settings")), "General", std::strlen("General")), "TempUnit", std::strlen("TempUnit"))->data.string_val.data;

  struct ads1118_get_temp_req_s adc_temp_req {
    .on_data_recvd = nullptr, .delay_fn = vTaskDelay, .value = &temperature
  };

  xSemaphoreTakeRecursive(thermometer_proc_sem, portMAX_DELAY);

  if (!thermometer_proc_running) {
    goto exit;
  }

  if ((rtc_fd = ::open(&sys, "rtc/rtc0", 3, 3u)) < 0) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = ::ioctl(&sys, "rtc/rtc0", rtc_ioctl_cmd_e::RTC_GET_TIME, &rtc_req, sizeof(rtc_req))) < 0) {
    if ((rtc_fd = ::close(&sys, "rtc/rtc0")) < 0) {
      brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rtc_fd = ::close(&sys, "rtc/rtc0")) < 0) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((adc_fd = ::open(&sys, "ads1118/ads1118_adc", 4, 3u)) < 0) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = ::ioctl(&sys, "ads1118/ads1118_adc", ads1118_ioctl_cmd_e::ADS1118_GET_TEMP_THERMISTOR, &adc_temp_req, sizeof(adc_temp_req))) < 0) {
    if ((rc = ::close(&sys, "ads1118/ads1118_adc")) < 0) {
      brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = ::close(&sys, "ads1118/ads1118_adc")) < 0) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  lcd_fb_clear(&lcd_fb);
  lcd_clear();

  std::memset(line, '\0', lcd_fb_x_size);
  std::snprintf(line, lcd_fb_x_size, "{%s}", cursor_entry_ptr->name);
  line_ptr = &line;
  lcd_fb_print_line(&lcd_fb, current_line++, &line_ptr, lcd_align_e::LCD_ALIGN_CENTER);
  std::memset(line, ' ', lcd_fb_x_size);
  lcd_fb_print_line(&lcd_fb, current_line++, &line_ptr, lcd_align_e::LCD_ALIGN_CENTER);

  if (!std::strcmp(temp_unit, "Celsius")) {

    std::snprintf(line, sizeof(line), "[%.1lf %cC]", kelvin_to_celsius_dbl(temperature), '\xdf');
  } else if (!std::strcmp(temp_unit, "Fahrenheit")) {

    std::snprintf(line, sizeof(line), "[%.1lf %cF]", kelvin_to_fahrenheit_dbl(temperature), '\xdf');
  } else if (!std::strcmp(temp_unit, "Kelvin")) {

    std::snprintf(line, sizeof(line), "[%.1lf %cK]", temperature, '\xdf');
  }

  lcd_fb_print_line(&lcd_fb, current_line++, &line_ptr, lcd_align_e::LCD_ALIGN_CENTER);

  std::snprintf(line, sizeof(line), "[%02u:%02u:%02u]", rtc.hour, rtc.min, rtc.sec);
  lcd_fb_print_line(&lcd_fb, current_line++, &line_ptr, lcd_align_e::LCD_ALIGN_CENTER);

  lcd_get_window(&lcd_fb, &window, 0u, 0u, lcd_align_e::LCD_ALIGN_CENTER);
  lcd_flush_fb_window(&window);

exit:
  xSemaphoreGiveRecursive(thermometer_proc_sem);
  return;
error:

  lcd_fb_clear(&lcd_fb);
  lcd_clear();

  std::memset(line, '\0', lcd_fb_x_size);
  std::snprintf(line, lcd_fb_x_size, "{%s}", cursor_entry_ptr->name);
  line_ptr = &line;
  lcd_fb_print_line(&lcd_fb, current_line++, &line_ptr, lcd_align_e::LCD_ALIGN_CENTER);
  std::memset(line, ' ', lcd_fb_x_size);
  lcd_fb_print_line(&lcd_fb, current_line++, &line_ptr, lcd_align_e::LCD_ALIGN_CENTER);

  if (!std::strcmp(temp_unit, "Celsius")) {

    std::snprintf(line, sizeof(line), "[???.? %cC]", '\xdf');
  } else if (!std::strcmp(temp_unit, "Fahrenheit")) {

    std::snprintf(line, sizeof(line), "[???.? %cF]", '\xdf');
  } else if (!std::strcmp(temp_unit, "Kelvin")) {

    std::snprintf(line, sizeof(line), "[???.? %cK]", '\xdf');
  }

  lcd_fb_print_line(&lcd_fb, current_line++, &line_ptr, lcd_align_e::LCD_ALIGN_CENTER);
  std::snprintf(line, sizeof(line), "[%02u:%02u:%02u]", rtc.hour, rtc.min, rtc.sec);
  lcd_fb_print_line(&lcd_fb, current_line++, &line_ptr, lcd_align_e::LCD_ALIGN_CENTER);

  lcd_get_window(&lcd_fb, &window, 0u, 0u, lcd_align_e::LCD_ALIGN_CENTER);
  lcd_flush_fb_window(&window);

  xSemaphoreGiveRecursive(thermometer_proc_sem);
  return;
}

static void thermometer_proc_adc_cbk(const void *data, size_t size) {}

static void brewery_proc_rtc_cbk(const void *data, size_t size) {
  int32_t rc, rtc_fd, adc_fd, time_min, time, time_left;
  double temperature;
  char line[lcd_fb_x_size];
  size_t current_line = 0u, margin = 0u;
  const char(*line_ptr)[lcd_fb_x_size];
  lcd_fb_window_ta window;
  struct rtc_s rtc_time, rtc_time_left;
  const struct rtc_get_time_req_s rtc_req { .rtc = &rtc_time };
  struct jfes_value *temp_unit;
  static int32_t temp_set;
  static bool temp_reached = false;
  bool heater_on;
  static constexpr const double temp_delta = 0.5f;

  static double error;
  static double pi_integral = 0.0f;
  static double pi_proportional;
  static uint32_t power_counter = 0u;
  static int32_t output_power;

  // Get PI coefficients
  const double p_k = jfes_get_child(jfes_get_child(jfes_get_child(menu, "Settings", std::strlen("Settings")), "PI", std::strlen("PI")), "P", std::strlen("P"))->data.double_val;
  const double i_k = jfes_get_child(jfes_get_child(jfes_get_child(menu, "Settings", std::strlen("Settings")), "PI", std::strlen("PI")), "I", std::strlen("I"))->data.double_val;

  const struct jfes_value *power_max = jfes_get_child(jfes_get_child(menu_variants, "Power(%)", std::strlen("Power(%)")), "max", std::strlen("max"));
  const struct jfes_value *power_step = jfes_get_child(jfes_get_child(menu_variants, "Power(%)", std::strlen("Power(%)")), "step", std::strlen("step"));

  struct ads1118_get_temp_req_s adc_req {
    .on_data_recvd = nullptr, .delay_fn = vTaskDelay, .value = &temperature
  };

  xSemaphoreTakeRecursive(brewery_proc_sem, portMAX_DELAY);

  if (!brewery_proc_running) {
    goto exit;
  }

  rtc_get_time(&rtc_time);
  // Get temperature
  if ((adc_fd = ::open(&sys, "ads1118/ads1118_adc", 4, 3u)) < 0) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = ::ioctl(&sys, "ads1118/ads1118_adc", ads1118_ioctl_cmd_e::ADS1118_GET_TEMP_THERMISTOR, &adc_req, sizeof(adc_req))) < 0) {
    if ((rc = ::close(&sys, "ads1118/ads1118_adc")) < 0) {
      brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = ::close(&sys, "ads1118/ads1118_adc")) < 0) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  // Calculate PI
  if (temp_set && !(brewery_proc_prev_state == BREWERY_PROC_STATE_UNDEFINED && brewery_proc_state == BREWERY_PROC_STATE_WAITING) && brewery_proc_state != BREWERY_PROC_STATE_BOILING) {
    error = (temp_set + temp_delta) - temperature;
    pi_integral += error * i_k;    // PI integral part
    pi_proportional = error * p_k; // PI proportional part

    for (double *const pid_parameter : {&pi_proportional, &pi_integral}) {
      if (*pid_parameter > static_cast<double>(power_max->data.int_val / power_step->data.int_val)) {
        *pid_parameter = static_cast<double>(power_max->data.int_val / power_step->data.int_val); // Restrict integral value by upper limit
      }
    }

    output_power = static_cast<uint32_t>(pi_integral + pi_proportional);

    if (output_power > power_max->data.int_val / power_step->data.int_val) {
      output_power = power_max->data.int_val / power_step->data.int_val; // Restrict integral value by upper limit
    }

    if (output_power < 0) {
      output_power = 0;
    }

    // Increment or reset power_counter variable
    if (power_counter < power_max->data.int_val / power_step->data.int_val) {
      power_counter++;
    } else {
      power_counter = 0u;
    }

    if (power_counter <= output_power && output_power) {
      heater_on = true;
      ac_load_on(heater_ac_load_num);
    } else {
      ac_load_off(heater_ac_load_num);
      heater_on = false;
    }
  }

  switch (brewery_proc_state) {
  case BREWERY_PROC_STATE_MASHING: {
    brewery_app_s::brewery_dbg_printfmt("MASHING STATE: [%02u:%02u:%02u]\r\n", rtc_time.hour, rtc_time.min, rtc_time.sec);
    temp_unit = jfes_get_child(jfes_get_child(jfes_get_child(menu, "Settings", std::strlen("Settings")), "General", std::strlen("General")), "TempUnit", std::strlen("TempUnit"));
    temp_set = jfes_get_child(jfes_get_child(brewery_recipe, "Mashing", std::strlen("Mashing")), "Temperature", std::strlen("Temperature"))->data.int_val;

    lcd_fb_clear(&lcd_fb);
    lcd_clear();

    std::memset(line, '\0', lcd_fb_x_size);
    std::snprintf(line, lcd_fb_x_size, "<%s>", brewery_recipe->name);
    line_ptr = &line;
    lcd_fb_print_line(&lcd_fb, current_line++, &line_ptr, lcd_align_e::LCD_ALIGN_CENTER);
    std::memset(line, '\0', lcd_fb_x_size);
    std::snprintf(line, lcd_fb_x_size, "Mashing | Heat. %s", heater_on ? "ON" : "OFF");
    lcd_fb_print_line(&lcd_fb, current_line++, &line_ptr, lcd_align_e::LCD_ALIGN_CENTER);

    if (!std::strcmp(temp_unit->data.string_val.data, "Celsius")) {

      std::snprintf(line, sizeof(line), "[%.1lf/%i %cC]", kelvin_to_celsius_dbl(temperature), kelvin_to_celsius(temp_set), '\xdf');
    } else if (!std::strcmp(temp_unit->data.string_val.data, "Fahrenheit")) {

      std::snprintf(line, sizeof(line), "[%.1lf/%i %cF]", kelvin_to_fahrenheit_dbl(temperature), kelvin_to_fahrenheit(temp_set), '\xdf');
    } else if (!std::strcmp(temp_unit->data.string_val.data, "Kelvin")) {

      std::snprintf(line, sizeof(line), "[%.1lf/%i %cK]", temperature, temp_set, '\xdf');
    }

    lcd_fb_print_line(&lcd_fb, current_line++, &line_ptr, lcd_align_e::LCD_ALIGN_CENTER);
    std::snprintf(line, sizeof(line), "[%02u:%02u:%02u, %i/%u%%]", rtc_time.hour, rtc_time.min, rtc_time.sec, output_power * power_step->data.int_val, power_max->data.int_val);
    lcd_fb_print_line(&lcd_fb, current_line++, &line_ptr, lcd_align_e::LCD_ALIGN_CENTER);

    lcd_get_window(&lcd_fb, &window, 0u, 0u, lcd_align_e::LCD_ALIGN_CENTER);
    lcd_flush_fb_window(&window);

    if (temperature >= temp_set) {
      brewery_proc_prev_state = BREWERY_PROC_STATE_MASHING;
      brewery_proc_next_state = static_cast<enum brewery_proc_state_e>(static_cast<uint32_t>(brewery_proc_state) + 1u);
      brewery_proc_state = BREWERY_PROC_STATE_WAITING;
      rtc_reset_time();
    }
  } break;

  case BREWERY_PROC_STATE_PAUSE1 ... BREWERY_PROC_STATE_PAUSE5: {
    const char *pause_names[] = {"Pause1", "Pause2", "Pause3", "Pause4", "Pause5"};
    const char *pause_name = pause_names[brewery_proc_state - BREWERY_PROC_STATE_PAUSE1];
    brewery_app_s::brewery_dbg_printfmt("PAUSE STATE: [%02u:%02u:%02u]\r\n", rtc_time.hour, rtc_time.min, rtc_time.sec);
    temp_unit = jfes_get_child(jfes_get_child(jfes_get_child(menu, "Settings", std::strlen("Settings")), "General", std::strlen("General")), "TempUnit", std::strlen("TempUnit"));
    temp_set = jfes_get_child(jfes_get_child(brewery_recipe, pause_name, std::strlen(pause_name)), "Temperature", std::strlen("Temperature"))->data.int_val;
    time_min = jfes_get_child(jfes_get_child(brewery_recipe, pause_name, std::strlen(pause_name)), "Time(min)", std::strlen("Time(min)"))->data.int_val;

    if (temperature >= temp_set && !temp_reached) {
      rtc_reset_time();
      rtc_get_time(&rtc_time);
      temp_reached = true;
    }

    time = time_min * 60u;
    time_left = time;

    if (temp_reached) {
      time_left = time - (rtc_time.sec + rtc_time.min * 60u + rtc_time.hour * 3600u);
    }

    rtc_time_left.sec = time_left % 60u;
    rtc_time_left.min = (time_left / 60u) % 60u;
    rtc_time_left.hour = (time_left / 60u) / 60u;

    lcd_fb_clear(&lcd_fb);
    lcd_clear();

    std::memset(line, '\0', lcd_fb_x_size);
    std::snprintf(line, lcd_fb_x_size, "<%s>", brewery_recipe->name);
    line_ptr = &line;
    lcd_fb_print_line(&lcd_fb, current_line++, &line_ptr, lcd_align_e::LCD_ALIGN_CENTER);
    std::memset(line, '\0', lcd_fb_x_size);
    std::snprintf(line, lcd_fb_x_size, "%s | Heat. %s", pause_name, heater_on ? "ON" : "OFF");
    lcd_fb_print_line(&lcd_fb, current_line++, &line_ptr, lcd_align_e::LCD_ALIGN_CENTER);

    if (!std::strcmp(temp_unit->data.string_val.data, "Celsius")) {

      std::snprintf(line, sizeof(line), "[%.1lf/%i %cC]", kelvin_to_celsius_dbl(temperature), kelvin_to_celsius(temp_set), '\xdf');
    } else if (!std::strcmp(temp_unit->data.string_val.data, "Fahrenheit")) {

      std::snprintf(line, sizeof(line), "[%.1lf/%i %cF]", kelvin_to_fahrenheit_dbl(temperature), kelvin_to_fahrenheit(temp_set), '\xdf');
    } else if (!std::strcmp(temp_unit->data.string_val.data, "Kelvin")) {

      std::snprintf(line, sizeof(line), "[%.1lf/%i %cK]", temperature, temp_set, '\xdf');
    }

    lcd_fb_print_line(&lcd_fb, current_line++, &line_ptr, lcd_align_e::LCD_ALIGN_CENTER);
    std::snprintf(line, sizeof(line), "[%02u:%02u:%02u, %i/%u%%]", rtc_time_left.hour, rtc_time_left.min, rtc_time_left.sec, output_power * power_step->data.int_val, power_max->data.int_val);
    lcd_fb_print_line(&lcd_fb, current_line++, &line_ptr, lcd_align_e::LCD_ALIGN_CENTER);

    lcd_get_window(&lcd_fb, &window, 0u, 0u, lcd_align_e::LCD_ALIGN_CENTER);
    lcd_flush_fb_window(&window);

    // Pause time is over
    if (!time_left) {
      brewery_proc_prev_state = brewery_proc_state;
      brewery_proc_state = static_cast<enum brewery_proc_state_e>(static_cast<uint32_t>(brewery_proc_state) + 1u);
      brewery_proc_next_state = static_cast<enum brewery_proc_state_e>(static_cast<uint32_t>(brewery_proc_state) + 1u);
      rtc_reset_time();
      temp_reached = false;
      temp_set = 0u;
      beep_short();
      user_action_led_on();
      vTaskDelay(50u);
      user_action_led_off();
      error = 0.0f;
      pi_integral = 0.0f;
    }
  } break;

  case BREWERY_PROC_STATE_MASHING_OUT: {
    brewery_app_s::brewery_dbg_printfmt("MASHOUT STATE: [%02u:%02u:%02u]\r\n", rtc_time.hour, rtc_time.min, rtc_time.sec);
    temp_unit = jfes_get_child(jfes_get_child(jfes_get_child(menu, "Settings", std::strlen("Settings")), "General", std::strlen("General")), "TempUnit", std::strlen("TempUnit"));
    temp_set = jfes_get_child(jfes_get_child(brewery_recipe, "MashOut", std::strlen("MashOut")), "Temperature", std::strlen("Temperature"))->data.int_val;
    time_min = jfes_get_child(jfes_get_child(brewery_recipe, "MashOut", std::strlen("MashOut")), "Time(min)", std::strlen("Time(min)"))->data.int_val;

    if (temperature >= temp_set && !temp_reached) {
      rtc_reset_time();
      rtc_get_time(&rtc_time);
      temp_reached = true;
    }

    time = time_min * 60u;
    time_left = time;

    if (temp_reached) {
      time_left = time - (rtc_time.sec + rtc_time.min * 60u + rtc_time.hour * 3600u);
    }

    rtc_time_left.sec = time_left % 60u;
    rtc_time_left.min = (time_left / 60u) % 60u;
    rtc_time_left.hour = (time_left / 60u) / 60u;

    lcd_fb_clear(&lcd_fb);
    lcd_clear();

    std::memset(line, '\0', lcd_fb_x_size);
    std::snprintf(line, lcd_fb_x_size, "<%s>", brewery_recipe->name);
    line_ptr = &line;
    lcd_fb_print_line(&lcd_fb, current_line++, &line_ptr, lcd_align_e::LCD_ALIGN_CENTER);
    std::memset(line, '\0', lcd_fb_x_size);
    std::snprintf(line, lcd_fb_x_size, "%s | Heat. %s", "MashOut", heater_on ? "ON" : "OFF");
    lcd_fb_print_line(&lcd_fb, current_line++, &line_ptr, lcd_align_e::LCD_ALIGN_CENTER);

    if (!std::strcmp(temp_unit->data.string_val.data, "Celsius")) {

      std::snprintf(line, sizeof(line), "[%.1lf/%i %cC]", kelvin_to_celsius_dbl(temperature), kelvin_to_celsius(temp_set), '\xdf');
    } else if (!std::strcmp(temp_unit->data.string_val.data, "Fahrenheit")) {

      std::snprintf(line, sizeof(line), "[%.1lf/%i %cF]", kelvin_to_fahrenheit_dbl(temperature), kelvin_to_fahrenheit(temp_set), '\xdf');
    } else if (!std::strcmp(temp_unit->data.string_val.data, "Kelvin")) {

      std::snprintf(line, sizeof(line), "[%.1lf/%i %cK]", temperature, temp_set, '\xdf');
    }

    lcd_fb_print_line(&lcd_fb, current_line++, &line_ptr, lcd_align_e::LCD_ALIGN_CENTER);
    std::snprintf(line, sizeof(line), "[%02u:%02u:%02u, %i/%u%%]", rtc_time_left.hour, rtc_time_left.min, rtc_time_left.sec, output_power * power_step->data.int_val, power_max->data.int_val);
    lcd_fb_print_line(&lcd_fb, current_line++, &line_ptr, lcd_align_e::LCD_ALIGN_CENTER);

    lcd_get_window(&lcd_fb, &window, 0u, 0u, lcd_align_e::LCD_ALIGN_CENTER);
    lcd_flush_fb_window(&window);

    // Pause time is over
    if (!time_left) {
      brewery_proc_prev_state = brewery_proc_state;
      brewery_proc_state = brewery_proc_state != BREWERY_PROC_STATE_MASHING_OUT ? static_cast<enum brewery_proc_state_e>(static_cast<uint32_t>(brewery_proc_state) + 1u) : BREWERY_PROC_STATE_WAITING;
      brewery_proc_next_state =
          brewery_proc_prev_state != BREWERY_PROC_STATE_MASHING_OUT ? static_cast<enum brewery_proc_state_e>(static_cast<uint32_t>(brewery_proc_state) + 1u) : BREWERY_PROC_STATE_BOILING;
      brewery_proc_boiling_state = BREWERY_PROC_BOILING_STATE_POWER_REG;
      rtc_reset_time();
      temp_reached = false;
      temp_set = 0u;
      error = 0.0f;
      pi_integral = 0.0f;
      output_power = 0;
    }
  } break;

  case BREWERY_PROC_STATE_BOILING: {
    bool heater_on;
    uint32_t time_passed = 0u;
    static uint32_t power_counter = 0u;
    static constexpr const uint32_t add_hop_time_delta_sec = 15u;
    const int32_t add_hop1_time_min = jfes_get_child(jfes_get_child(brewery_recipe, "AddHop", std::strlen("AddHop")), "AddHopTime1(min)", std::strlen("AddHopTime1(min)"))->data.int_val;
    const int32_t add_hop2_time_min = jfes_get_child(jfes_get_child(brewery_recipe, "AddHop", std::strlen("AddHop")), "AddHopTime2(min)", std::strlen("AddHopTime2(min)"))->data.int_val;
    const int32_t add_hop3_time_min = jfes_get_child(jfes_get_child(brewery_recipe, "AddHop", std::strlen("AddHop")), "AddHopTime3(min)", std::strlen("AddHopTime3(min)"))->data.int_val;
    const int32_t add_hop4_time_min = jfes_get_child(jfes_get_child(brewery_recipe, "AddHop", std::strlen("AddHop")), "AddHopTime4(min)", std::strlen("AddHopTime4(min)"))->data.int_val;
    const int32_t output_power = jfes_get_child(jfes_get_child(brewery_recipe, "Boiling", std::strlen("Boiling")), "Power(%)", std::strlen("Power(%)"))->data.int_val;

    const struct jfes_value *power_max = jfes_get_child(jfes_get_child(menu_variants, "Power(%)", std::strlen("Power(%)")), "max", std::strlen("max"));
    const struct jfes_value *power_step = jfes_get_child(jfes_get_child(menu_variants, "Power(%)", std::strlen("Power(%)")), "step", std::strlen("step"));

    brewery_app_s::brewery_dbg_printfmt("BOILING STATE: [%02u:%02u:%02u]\r\n", rtc_time.hour, rtc_time.min, rtc_time.sec);
    temp_unit = jfes_get_child(jfes_get_child(jfes_get_child(menu, "Settings", std::strlen("Settings")), "General", std::strlen("General")), "TempUnit", std::strlen("TempUnit"));
    temp_set = jfes_get_child(jfes_get_child(brewery_recipe, "Boiling", std::strlen("Boiling")), "Temperature", std::strlen("Temperature"))->data.int_val;
    time_min = jfes_get_child(jfes_get_child(brewery_recipe, "Boiling", std::strlen("Boiling")), "Time(min)", std::strlen("Time(min)"))->data.int_val;
    time = time_min * 60u;
    time_left = time;

    // Increment or reset power_counter variable
    if (power_counter < power_max->data.int_val / power_step->data.int_val) {
      power_counter++;
    } else {
      power_counter = 0u;
    }

    if (temperature >= temp_set - temp_delta && !temp_reached) {
      rtc_reset_time();
      rtc_get_time(&rtc_time);
      temp_reached = true;
    }

    if (power_counter <= output_power / power_step->data.int_val && output_power && temperature < temp_set - temp_delta) {
      ac_load_on(heater_ac_load_num);
      heater_on = true;
    } else {
      ac_load_off(heater_ac_load_num);
      heater_on = false;
    }

    if (temp_reached) {
      time_passed = rtc_time.sec + rtc_time.min * 60u + rtc_time.hour * 3600u;
      time_left = time - time_passed;
    }

    rtc_time_left.sec = time_left % 60u;
    rtc_time_left.min = (time_left / 60u) % 60u;
    rtc_time_left.hour = (time_left / 60u) / 60u;

    // Add hop 1 time reached
    if (add_hop1_time_min && time_passed && (time_passed >= (add_hop1_time_min * 60u) - add_hop_time_delta_sec && time_passed <= add_hop1_time_min * 60u)) {
      user_action_led_on();

      lcd_fb_clear(&lcd_fb);
      std::memset(line, '\0', lcd_fb_x_size);
      std::snprintf(line, lcd_fb_x_size, "<%s>", brewery_recipe->name);
      line_ptr = &line;
      lcd_fb_print_line(&lcd_fb, current_line++, &line_ptr, lcd_align_e::LCD_ALIGN_CENTER);
      std::memset(line, '\0', lcd_fb_x_size);
      std::snprintf(line, lcd_fb_x_size, "%s | Heat. %s", "Boiling", heater_on ? "ON" : "OFF");
      lcd_fb_print_line(&lcd_fb, current_line++, &line_ptr, lcd_align_e::LCD_ALIGN_CENTER);
      std::memset(line, '\0', lcd_fb_x_size);
      std::snprintf(line, lcd_fb_x_size, "Add Hop #1");
      lcd_fb_print_line(&lcd_fb, current_line++, &line_ptr, lcd_align_e::LCD_ALIGN_CENTER);
      std::snprintf(line, sizeof(line), "[%02u:%02u:%02u,%c%i/%u%%]", rtc_time_left.hour, rtc_time_left.min, rtc_time_left.sec,
                    brewery_proc_boiling_state == BREWERY_PROC_BOILING_STATE_POWER_REG ? '\x7e' : ' ', temperature < temp_set - temp_delta ? output_power : 0, max_power_percent);
      lcd_fb_print_line(&lcd_fb, current_line++, &line_ptr, lcd_align_e::LCD_ALIGN_CENTER);
      lcd_get_window(&lcd_fb, &window, 0u, 0u, lcd_align_e::LCD_ALIGN_CENTER);

      lcd_clear();
      lcd_flush_fb_window(&window);

      if (beep_cnt < 3u) {
        beep();
        beep_cnt++;
      }

      vTaskDelay(50u);
      user_action_led_off();

      // Add hop 2 time reached
    } else if (add_hop2_time_min && time_passed && (time_passed >= (add_hop2_time_min * 60u) - add_hop_time_delta_sec && time_passed <= add_hop2_time_min * 60u)) {
      user_action_led_on();

      lcd_fb_clear(&lcd_fb);
      std::memset(line, '\0', lcd_fb_x_size);
      std::snprintf(line, lcd_fb_x_size, "<%s>", brewery_recipe->name);
      line_ptr = &line;
      lcd_fb_print_line(&lcd_fb, current_line++, &line_ptr, lcd_align_e::LCD_ALIGN_CENTER);
      std::memset(line, '\0', lcd_fb_x_size);
      std::snprintf(line, lcd_fb_x_size, "%s | Heat. %s", "Boiling", heater_on ? "ON" : "OFF");
      lcd_fb_print_line(&lcd_fb, current_line++, &line_ptr, lcd_align_e::LCD_ALIGN_CENTER);
      std::memset(line, '\0', lcd_fb_x_size);
      std::snprintf(line, lcd_fb_x_size, "Add Hop #2");
      lcd_fb_print_line(&lcd_fb, current_line++, &line_ptr, lcd_align_e::LCD_ALIGN_CENTER);
      std::snprintf(line, sizeof(line), "[%02u:%02u:%02u,%c%i/%u%%]", rtc_time_left.hour, rtc_time_left.min, rtc_time_left.sec,
                    brewery_proc_boiling_state == BREWERY_PROC_BOILING_STATE_POWER_REG ? '\x7e' : ' ', temperature < temp_set - temp_delta ? output_power : 0, max_power_percent);
      lcd_fb_print_line(&lcd_fb, current_line++, &line_ptr, lcd_align_e::LCD_ALIGN_CENTER);
      lcd_get_window(&lcd_fb, &window, 0u, 0u, lcd_align_e::LCD_ALIGN_CENTER);

      lcd_clear();
      lcd_flush_fb_window(&window);

      if (beep_cnt < 3u) {
        beep();
        beep_cnt++;
      }

      vTaskDelay(50u);
      user_action_led_off();

      // Add hop 3 time reached
    } else if (add_hop3_time_min && time_passed && (time_passed >= (add_hop3_time_min * 60u) - add_hop_time_delta_sec && time_passed <= add_hop3_time_min * 60u)) {
      user_action_led_on();

      lcd_fb_clear(&lcd_fb);
      std::memset(line, '\0', lcd_fb_x_size);
      std::snprintf(line, lcd_fb_x_size, "<%s>", brewery_recipe->name);
      line_ptr = &line;
      lcd_fb_print_line(&lcd_fb, current_line++, &line_ptr, lcd_align_e::LCD_ALIGN_CENTER);
      std::memset(line, '\0', lcd_fb_x_size);
      std::snprintf(line, lcd_fb_x_size, "%s | Heat. %s", "Boiling", heater_on ? "ON" : "OFF");
      lcd_fb_print_line(&lcd_fb, current_line++, &line_ptr, lcd_align_e::LCD_ALIGN_CENTER);
      std::memset(line, '\0', lcd_fb_x_size);
      std::snprintf(line, lcd_fb_x_size, "Add Hop #3");
      lcd_fb_print_line(&lcd_fb, current_line++, &line_ptr, lcd_align_e::LCD_ALIGN_CENTER);
      std::snprintf(line, sizeof(line), "[%02u:%02u:%02u,%c%i/%u%%]", rtc_time_left.hour, rtc_time_left.min, rtc_time_left.sec,
                    brewery_proc_boiling_state == BREWERY_PROC_BOILING_STATE_POWER_REG ? '\x7e' : ' ', temperature < temp_set - temp_delta ? output_power : 0, max_power_percent);
      lcd_fb_print_line(&lcd_fb, current_line++, &line_ptr, lcd_align_e::LCD_ALIGN_CENTER);
      lcd_get_window(&lcd_fb, &window, 0u, 0u, lcd_align_e::LCD_ALIGN_CENTER);

      lcd_clear();
      lcd_flush_fb_window(&window);

      if (beep_cnt < 3u) {
        beep();
        beep_cnt++;
      }

      vTaskDelay(50u);
      user_action_led_off();

      // Add hop 4 time reached
    } else if (add_hop4_time_min && time_passed && (time_passed >= (add_hop4_time_min * 60u) - add_hop_time_delta_sec && time_passed <= add_hop4_time_min * 60u)) {
      user_action_led_on();

      lcd_fb_clear(&lcd_fb);
      std::memset(line, '\0', lcd_fb_x_size);
      std::snprintf(line, lcd_fb_x_size, "<%s>", brewery_recipe->name);
      line_ptr = &line;
      lcd_fb_print_line(&lcd_fb, current_line++, &line_ptr, lcd_align_e::LCD_ALIGN_CENTER);
      std::memset(line, '\0', lcd_fb_x_size);
      std::snprintf(line, lcd_fb_x_size, "%s | Heat. %s", "Boiling", heater_on ? "ON" : "OFF");
      lcd_fb_print_line(&lcd_fb, current_line++, &line_ptr, lcd_align_e::LCD_ALIGN_CENTER);
      std::memset(line, '\0', lcd_fb_x_size);
      std::snprintf(line, lcd_fb_x_size, "Add Hop #4");
      lcd_fb_print_line(&lcd_fb, current_line++, &line_ptr, lcd_align_e::LCD_ALIGN_CENTER);
      std::snprintf(line, sizeof(line), "[%02u:%02u:%02u,%c%i/%u%%]", rtc_time_left.hour, rtc_time_left.min, rtc_time_left.sec,
                    brewery_proc_boiling_state == BREWERY_PROC_BOILING_STATE_POWER_REG ? '\x7e' : ' ', temperature < temp_set - temp_delta ? output_power : 0, max_power_percent);
      lcd_fb_print_line(&lcd_fb, current_line++, &line_ptr, lcd_align_e::LCD_ALIGN_CENTER);
      lcd_get_window(&lcd_fb, &window, 0u, 0u, lcd_align_e::LCD_ALIGN_CENTER);

      lcd_clear();
      lcd_flush_fb_window(&window);

      if (beep_cnt < 3u) {
        beep();
        beep_cnt++;
      }

      vTaskDelay(50u);
      user_action_led_off();
    } else {

      lcd_fb_clear(&lcd_fb);
      lcd_clear();
      std::memset(line, '\0', lcd_fb_x_size);
      std::snprintf(line, lcd_fb_x_size, "<%s>", brewery_recipe->name);
      line_ptr = &line;
      lcd_fb_print_line(&lcd_fb, current_line++, &line_ptr, lcd_align_e::LCD_ALIGN_CENTER);
      std::memset(line, '\0', lcd_fb_x_size);
      std::snprintf(line, lcd_fb_x_size, "%s | Heat. %s", "Boiling", heater_on ? "ON" : "OFF");
      lcd_fb_print_line(&lcd_fb, current_line++, &line_ptr, lcd_align_e::LCD_ALIGN_CENTER);

      if (!std::strcmp(temp_unit->data.string_val.data, "Celsius")) {

        std::snprintf(line, sizeof(line), "%c[%.1lf/%i%cC]", brewery_proc_boiling_state == BREWERY_PROC_BOILING_STATE_TEMP_REG ? '\x7e' : ' ', kelvin_to_celsius_dbl(temperature),
                      kelvin_to_celsius(temp_set), '\xdf');
      } else if (!std::strcmp(temp_unit->data.string_val.data, "Fahrenheit")) {

        std::snprintf(line, sizeof(line), "%c[%.1lf/%i%cF]", brewery_proc_boiling_state == BREWERY_PROC_BOILING_STATE_TEMP_REG ? '\x7e' : ' ', kelvin_to_fahrenheit_dbl(temperature),
                      kelvin_to_fahrenheit(temp_set), '\xdf');
      } else if (!std::strcmp(temp_unit->data.string_val.data, "Kelvin")) {

        std::snprintf(line, sizeof(line), "%c[%.1lf/%i%cK]", brewery_proc_boiling_state == BREWERY_PROC_BOILING_STATE_TEMP_REG ? '\x7e' : ' ', temperature, temp_set, '\xdf');
      }

      lcd_fb_print_line(&lcd_fb, current_line++, &line_ptr, lcd_align_e::LCD_ALIGN_CENTER);

      std::snprintf(line, sizeof(line), "[%02u:%02u:%02u,%c%i/%u%%]", rtc_time_left.hour, rtc_time_left.min, rtc_time_left.sec,
                    brewery_proc_boiling_state == BREWERY_PROC_BOILING_STATE_POWER_REG ? '\x7e' : ' ', temperature < temp_set - temp_delta ? output_power : 0, max_power_percent);
      lcd_fb_print_line(&lcd_fb, current_line++, &line_ptr, lcd_align_e::LCD_ALIGN_CENTER);
      lcd_get_window(&lcd_fb, &window, 0u, 0u, lcd_align_e::LCD_ALIGN_CENTER);
      lcd_flush_fb_window(&window);
      beep_cnt = 0u;
    }

    // Boiling time is over
    if (!time_left) {
      brewery_proc_prev_state = brewery_proc_state;
      brewery_proc_state = BREWERY_PROC_STATE_WAITING;
      brewery_proc_next_state = BREWERY_PROC_STATE_END;
      rtc_reset_time();
      temp_reached = false;
      temp_set = 0u;
    }
  } break;

  case BREWERY_PROC_STATE_WAITING: {
    switch (brewery_proc_prev_state) {
    case BREWERY_PROC_STATE_MASHING: {
      const char *next_step_name = "Pause1";
      temp_unit = jfes_get_child(jfes_get_child(jfes_get_child(menu, "Settings", std::strlen("Settings")), "General", std::strlen("General")), "TempUnit", std::strlen("TempUnit"));
      temp_set = jfes_get_child(jfes_get_child(brewery_recipe, "Mashing", std::strlen("Mashing")), "Temperature", std::strlen("Temperature"))->data.int_val;

      lcd_fb_clear(&lcd_fb);
      lcd_clear();

      std::memset(line, '\0', lcd_fb_x_size);
      std::snprintf(line, lcd_fb_x_size, "<%s>", brewery_recipe->name);
      line_ptr = &line;
      lcd_fb_print_line(&lcd_fb, current_line++, &line_ptr, lcd_align_e::LCD_ALIGN_CENTER);
      std::memset(line, '\0', lcd_fb_x_size);
      std::snprintf(line, lcd_fb_x_size, "Waiting for %s", next_step_name);
      lcd_fb_print_line(&lcd_fb, current_line++, &line_ptr, lcd_align_e::LCD_ALIGN_CENTER);

      if (!std::strcmp(temp_unit->data.string_val.data, "Celsius")) {

        std::snprintf(line, sizeof(line), "[%.1lf/%i %cC]", kelvin_to_celsius_dbl(temperature), kelvin_to_celsius(temp_set), '\xdf');
      } else if (!std::strcmp(temp_unit->data.string_val.data, "Fahrenheit")) {

        std::snprintf(line, sizeof(line), "[%.1lf/%i %cF]", kelvin_to_fahrenheit_dbl(temperature), kelvin_to_fahrenheit(temp_set), '\xdf');
      } else if (!std::strcmp(temp_unit->data.string_val.data, "Kelvin")) {

        std::snprintf(line, sizeof(line), "[%.1lf/%i %cK]", temperature, temp_set, '\xdf');
      }

      lcd_fb_print_line(&lcd_fb, current_line++, &line_ptr, lcd_align_e::LCD_ALIGN_CENTER);
      std::snprintf(line, sizeof(line), "[%02u:%02u:%02u, %i/%u%%]", rtc_time.hour, rtc_time.min, rtc_time.sec, output_power * power_step->data.int_val, power_max->data.int_val);
      lcd_fb_print_line(&lcd_fb, current_line++, &line_ptr, lcd_align_e::LCD_ALIGN_CENTER);

      lcd_get_window(&lcd_fb, &window, 0u, 0u, lcd_align_e::LCD_ALIGN_CENTER);
      lcd_flush_fb_window(&window);

      if (beep_cnt < 3u) {
        beep();
        beep_cnt++;
      }

      user_action_led_on();
    } break;

    case BREWERY_PROC_STATE_PAUSE1 ... BREWERY_PROC_STATE_MASHING_OUT: {
      const char *next_step_name = "Boiling";

      temp_unit = jfes_get_child(jfes_get_child(jfes_get_child(menu, "Settings", std::strlen("Settings")), "General", std::strlen("General")), "TempUnit", std::strlen("TempUnit"));
      lcd_fb_clear(&lcd_fb);
      lcd_clear();

      std::memset(line, '\0', lcd_fb_x_size);
      std::snprintf(line, lcd_fb_x_size, "<%s>", brewery_recipe->name);
      line_ptr = &line;
      lcd_fb_print_line(&lcd_fb, current_line++, &line_ptr, lcd_align_e::LCD_ALIGN_CENTER);
      std::memset(line, '\0', lcd_fb_x_size);
      std::snprintf(line, lcd_fb_x_size, "Waiting for %s", next_step_name);
      lcd_fb_print_line(&lcd_fb, current_line++, &line_ptr, lcd_align_e::LCD_ALIGN_CENTER);

      if (!std::strcmp(temp_unit->data.string_val.data, "Celsius")) {

        std::snprintf(line, sizeof(line), "[%.1lf %cC]", kelvin_to_celsius_dbl(temperature), '\xdf');
      } else if (!std::strcmp(temp_unit->data.string_val.data, "Fahrenheit")) {

        std::snprintf(line, sizeof(line), "[%.1lf %cF]", kelvin_to_fahrenheit_dbl(temperature), '\xdf');
      } else if (!std::strcmp(temp_unit->data.string_val.data, "Kelvin")) {

        std::snprintf(line, sizeof(line), "[%.1lf %cK]", temperature, '\xdf');
      }

      lcd_fb_print_line(&lcd_fb, current_line++, &line_ptr, lcd_align_e::LCD_ALIGN_CENTER);
      std::snprintf(line, sizeof(line), "[%02u:%02u:%02u, %i/%u%%]", rtc_time.hour, rtc_time.min, rtc_time.sec, output_power * power_step->data.int_val, power_max->data.int_val);
      lcd_fb_print_line(&lcd_fb, current_line++, &line_ptr, lcd_align_e::LCD_ALIGN_CENTER);
      lcd_get_window(&lcd_fb, &window, 0u, 0u, lcd_align_e::LCD_ALIGN_CENTER);
      lcd_flush_fb_window(&window);

      if (beep_cnt < 3u) {
        beep();
        beep_cnt++;
      }

      user_action_led_on();
    } break;

    case BREWERY_PROC_STATE_BOILING: {
      lcd_fb_clear(&lcd_fb);
      lcd_clear();
      std::memset(line, '\0', lcd_fb_x_size);
      std::snprintf(line, lcd_fb_x_size, "<%s>", brewery_recipe->name);
      line_ptr = &line;
      lcd_fb_print_line(&lcd_fb, current_line++, &line_ptr, lcd_align_e::LCD_ALIGN_CENTER);
      std::memset(line, '\0', lcd_fb_x_size);
      std::snprintf(line, lcd_fb_x_size, "Brewing completion");
      lcd_fb_print_line(&lcd_fb, current_line++, &line_ptr, lcd_align_e::LCD_ALIGN_CENTER);
      std::memset(line, ' ', lcd_fb_x_size);
      lcd_fb_print_line(&lcd_fb, current_line++, &line_ptr, lcd_align_e::LCD_ALIGN_CENTER);
      std::snprintf(line, sizeof(line), "[%02u:%02u:%02u, %i/%u%%]", rtc_time.hour, rtc_time.min, rtc_time.sec, output_power * power_step->data.int_val, power_max->data.int_val);
      lcd_fb_print_line(&lcd_fb, current_line++, &line_ptr, lcd_align_e::LCD_ALIGN_CENTER);
      lcd_get_window(&lcd_fb, &window, 0u, 0u, lcd_align_e::LCD_ALIGN_CENTER);
      lcd_flush_fb_window(&window);

      if (beep_cnt < 3u) {
        beep();
        beep_cnt++;
      }

      user_action_led_on();
    } break;

    case BREWERY_PROC_STATE_UNDEFINED: {
      lcd_fb_clear(&lcd_fb);
      lcd_clear();
      temp_set = 0u;
      std::memset(line, '\0', lcd_fb_x_size);
      std::snprintf(line, lcd_fb_x_size, "<%s>", brewery_recipe->name);
      line_ptr = &line;
      lcd_fb_print_line(&lcd_fb, current_line++, &line_ptr, lcd_align_e::LCD_ALIGN_CENTER);
      std::memset(line, ' ', lcd_fb_x_size);
      lcd_fb_print_line(&lcd_fb, current_line++, &line_ptr, lcd_align_e::LCD_ALIGN_CENTER);
      std::snprintf(line, lcd_fb_x_size, "Hold SET button");
      lcd_fb_print_line(&lcd_fb, current_line++, &line_ptr, lcd_align_e::LCD_ALIGN_CENTER);
      std::snprintf(line, sizeof(line), "To start!");
      lcd_fb_print_line(&lcd_fb, current_line++, &line_ptr, lcd_align_e::LCD_ALIGN_CENTER);
      lcd_get_window(&lcd_fb, &window, 0u, 0u, lcd_align_e::LCD_ALIGN_CENTER);
      lcd_flush_fb_window(&window);

      if (beep_cnt < 3u) {
        beep();
        beep_cnt++;
      }

      user_action_led_on();
    } break;
    default:
      break;
    }

    brewery_app_s::brewery_dbg_printfmt("WAITING STATE: [%02u:%02u:%02u]\r\n", rtc_time.hour, rtc_time.min, rtc_time.sec);
  } break;

  case BREWERY_PROC_STATE_UNDEFINED: {
    brewery_app_s::brewery_dbg_printfmt("UNDEFINED STATE: [%02u:%02u:%02u]\r\n", rtc_time.hour, rtc_time.min, rtc_time.sec);
  } break;

  case BREWERY_PROC_STATE_END: {
    xSemaphoreGiveRecursive(brewery_proc_sem);
    user_action_led_off();
    ac_load_off(heater_ac_load_num);
    brewery_proc_next_state = BREWERY_PROC_STATE_UNDEFINED;
    brewery_proc_stop();
    trigger_json_val(&lcd_fb, cursor_entry_ptr->parent, cursor_entry_ptr->parent->name ? cursor_entry_ptr->parent->name : application_name);
    state = MENU_MODE_ENTRIES_SURFING;
    return;
  } break;
  default:
    break;
  }

exit:
  xSemaphoreGiveRecursive(brewery_proc_sem);
  return;
error:

  user_action_led_off();
  ac_load_off(heater_ac_load_num);
  brewery_proc_next_state = BREWERY_PROC_STATE_UNDEFINED;
  brewery_proc_stop();
  trigger_json_val(&lcd_fb, cursor_entry_ptr->parent, cursor_entry_ptr->parent->name ? cursor_entry_ptr->parent->name : application_name);
  state = MENU_MODE_ENTRIES_SURFING;
  xSemaphoreGiveRecursive(brewery_proc_sem);
  return;
}

static void brewery_proc_adc_cbk(const void *data, size_t size) {}

static void rtc_reset_time() {
  int32_t rc, rtc_fd;

  struct rtc_s rtc_time {
    .hour = 0u, .min = 0u, .sec = 0u
  };

  struct rtc_set_time_req_s rtc_time_req {
    .rtc = &rtc_time
  };

  if ((rtc_fd = ::open(&sys, "rtc/rtc0", 3, 3u)) < 0) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto exit;
  }

  if ((rc = ::ioctl(&sys, "rtc/rtc0", rtc_ioctl_cmd_e::RTC_SET_TIME, &rtc_time_req, sizeof(rtc_time_req))) < 0) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto exit;
  }

exit:
  if ((rc = ::close(&sys, "rtc/rtc0")) < 0) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
  }
}

static void rtc_set_time(const struct rtc_s *rtc_time) {
  int32_t rc, rtc_fd;

  struct rtc_set_time_req_s rtc_time_req {
    .rtc = rtc_time
  };

  if ((rtc_fd = ::open(&sys, "rtc/rtc0", 3, 3u)) < 0) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto exit;
  }

  if ((rc = ::ioctl(&sys, "rtc/rtc0", rtc_ioctl_cmd_e::RTC_SET_TIME, &rtc_time_req, sizeof(rtc_time_req))) < 0) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto exit;
  }

exit:
  if ((rc = ::close(&sys, "rtc/rtc0")) < 0) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
  }
}

static void rtc_get_time(struct rtc_s *const rtc_time) {
  int32_t rtc_fd, rc;
  const struct rtc_get_time_req_s rtc_req { .rtc = rtc_time };

  // Get time
  if ((rtc_fd = ::open(&sys, "rtc/rtc0", 3, 3u)) < 0) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = ::ioctl(&sys, "rtc/rtc0", rtc_ioctl_cmd_e::RTC_GET_TIME, &rtc_req, sizeof(rtc_req))) < 0) {
    if ((rtc_fd = ::close(&sys, "rtc/rtc0")) < 0) {
      brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rtc_fd = ::close(&sys, "rtc/rtc0")) < 0) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

exit:
  return;
error:
  return;
}

static void temp_hold_proc_start() {
  int32_t rc, rtc_fd, adc_fd;

  struct rtc_s rtc_time {
    .hour = 0u, .min = 0u, .sec = 0u
  };

  struct rtc_set_time_req_s rtc_time_req {
    .rtc = &rtc_time
  };

  struct rtc_callback_req_s rtc_cbk_req {
    .callback = rtc_callback
  };

  struct rtc_irq_req_s rtc_irq_req {
    .priority = 5u
  };

  if ((rtc_fd = ::open(&sys, "rtc/rtc0", 3, 3u)) < 0) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto exit;
  }

  if ((rc = ::ioctl(&sys, "rtc/rtc0", rtc_ioctl_cmd_e::RTC_SET_TIME, &rtc_time_req, sizeof(rtc_time_req))) < 0) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto exit;
  }

  if ((rc = ::ioctl(&sys, "rtc/rtc0", rtc_ioctl_cmd_e::RTC_ON_SECOND_SET, &rtc_cbk_req, sizeof(rtc_cbk_req))) < 0) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto exit;
  }

  if ((rc = ::ioctl(&sys, "rtc/rtc0", rtc_ioctl_cmd_e::RTC_IRQ_ENABLE, &rtc_irq_req, sizeof(rtc_irq_req))) < 0) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto exit;
  }

  if ((adc_fd = ::open(&sys, "ads1118/ads1118_adc", 3, 3u)) < 0) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto exit;
  }

exit:
  if ((rc = ::close(&sys, "ads1118/ads1118_adc")) < 0) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
  }

  if ((rc = ::close(&sys, "rtc/rtc0")) < 0) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
  }

  beep_short();
  temp_hold_proc_running = true;
  return;
}

static void temp_hold_timer_proc_start() {
  int32_t rc, rtc_fd, adc_fd;

  struct rtc_s rtc_time {
    .hour = 0u, .min = 0u, .sec = 0u
  };

  struct rtc_set_time_req_s rtc_time_req {
    .rtc = &rtc_time
  };

  struct rtc_callback_req_s rtc_cbk_req {
    .callback = rtc_callback
  };

  struct rtc_irq_req_s rtc_irq_req {
    .priority = 5u
  };

  if ((rtc_fd = ::open(&sys, "rtc/rtc0", 3, 3u)) < 0) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto exit;
  }

  if ((rc = ::ioctl(&sys, "rtc/rtc0", rtc_ioctl_cmd_e::RTC_SET_TIME, &rtc_time_req, sizeof(rtc_time_req))) < 0) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto exit;
  }

  if ((rc = ::ioctl(&sys, "rtc/rtc0", rtc_ioctl_cmd_e::RTC_ON_SECOND_SET, &rtc_cbk_req, sizeof(rtc_cbk_req))) < 0) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto exit;
  }

  if ((rc = ::ioctl(&sys, "rtc/rtc0", rtc_ioctl_cmd_e::RTC_IRQ_ENABLE, &rtc_irq_req, sizeof(rtc_irq_req))) < 0) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto exit;
  }

  if ((adc_fd = ::open(&sys, "ads1118/ads1118_adc", 3, 3u)) < 0) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto exit;
  }

exit:
  if ((rc = ::close(&sys, "ads1118/ads1118_adc")) < 0) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
  }

  if ((rc = ::close(&sys, "rtc/rtc0")) < 0) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
  }

  beep_short();
  temp_hold_timer_proc_running = true;
  return;
}

static void temp_hold_proc_stop() {
  int32_t rc, rtc_fd, adc_fd;
  xSemaphoreTakeRecursive(temp_hold_proc_sem, portMAX_DELAY);
exit:

  beep_short();
  ac_load_off(heater_ac_load_num);
  temp_hold_proc_running = false;
  xSemaphoreGiveRecursive(temp_hold_proc_sem);
}

static void temp_hold_timer_proc_stop() {
  int32_t rc, rtc_fd, adc_fd;
  xSemaphoreTakeRecursive(temp_hold_timer_proc_sem, portMAX_DELAY);
exit:

  beep_short();
  ac_load_off(heater_ac_load_num);
  temp_hold_timer_proc_running = false;
  xSemaphoreGiveRecursive(temp_hold_timer_proc_sem);
}

static void temp_hold_proc_adc_cbk(const void *data, size_t size) {}
static void temp_hold_timer_proc_adc_cbk(const void *data, size_t size) {}

static void temp_hold_proc_rtc_cbk(const void *data, size_t size) {
  const size_t seconds = *static_cast<const size_t *>(data);
  int32_t rc, adc_fd, rtc_fd;
  uint16_t adc_val;
  double temperature;
  char line[lcd_fb_x_size];
  size_t current_line = 0u, margin = 0u;
  const char(*line_ptr)[lcd_fb_x_size];
  lcd_fb_window_ta window;
  struct rtc_s rtc;
  bool heater_on;

  static double error;
  static double pi_integral = 0.0f;
  static double pi_proportional;
  static uint32_t power_counter = 0u;
  static int32_t output_power;

  // Get PI coefficients
  const double p_k = jfes_get_child(jfes_get_child(jfes_get_child(menu, "Settings", std::strlen("Settings")), "PI", std::strlen("PI")), "P", std::strlen("P"))->data.double_val;
  const double i_k = jfes_get_child(jfes_get_child(jfes_get_child(menu, "Settings", std::strlen("Settings")), "PI", std::strlen("PI")), "I", std::strlen("I"))->data.double_val;

  const struct jfes_value *power_max = jfes_get_child(jfes_get_child(menu_variants, "Power(%)", std::strlen("Power(%)")), "max", std::strlen("max"));
  const struct jfes_value *power_step = jfes_get_child(jfes_get_child(menu_variants, "Power(%)", std::strlen("Power(%)")), "step", std::strlen("step"));

  const struct rtc_get_time_req_s rtc_req { .rtc = &rtc };

  const char *temp_unit =
      jfes_get_child(jfes_get_child(jfes_get_child(menu, "Settings", std::strlen("Settings")), "General", std::strlen("General")), "TempUnit", std::strlen("TempUnit"))->data.string_val.data;

  const int32_t temp_set = jfes_get_child(jfes_get_child(menu, "TempHold", std::strlen("TempHold")), "Temperature", std::strlen("Temperature"))->data.int_val;
  const int32_t pwr_set = jfes_get_child(jfes_get_child(menu, "TempHold", std::strlen("TempHold")), "Power(%)", std::strlen("Power(%)"))->data.int_val;

  struct ads1118_get_temp_req_s adc_temp_req {
    .on_data_recvd = nullptr, .delay_fn = vTaskDelay, .value = &temperature
  };

  xSemaphoreTakeRecursive(temp_hold_proc_sem, portMAX_DELAY);

  if (!temp_hold_proc_running) {
    goto exit;
  }

  if ((rtc_fd = ::open(&sys, "rtc/rtc0", 3, 3u)) < 0) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = ::ioctl(&sys, "rtc/rtc0", rtc_ioctl_cmd_e::RTC_GET_TIME, &rtc_req, sizeof(rtc_req))) < 0) {
    if ((rtc_fd = ::close(&sys, "rtc/rtc0")) < 0) {
      brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rtc_fd = ::close(&sys, "rtc/rtc0")) < 0) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((adc_fd = ::open(&sys, "ads1118/ads1118_adc", 4, 3u)) < 0) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = ::ioctl(&sys, "ads1118/ads1118_adc", ads1118_ioctl_cmd_e::ADS1118_GET_TEMP_THERMISTOR, &adc_temp_req, sizeof(adc_temp_req))) < 0) {
    if ((rc = ::close(&sys, "ads1118/ads1118_adc")) < 0) {
      brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = ::close(&sys, "ads1118/ads1118_adc")) < 0) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  // Calculate PI
  if (temp_set) {
    error = temp_set - temperature;
    pi_integral += error * i_k;    // PI integral part
    pi_proportional = error * p_k; // PI proportional part

    for (double *const pid_parameter : {&pi_proportional, &pi_integral}) {
      if (*pid_parameter > static_cast<double>(power_max->data.int_val / power_step->data.int_val)) {
        *pid_parameter = static_cast<double>(power_max->data.int_val / power_step->data.int_val); // Restrict integral value by upper limit
      }
    }

    output_power = static_cast<uint32_t>(pi_integral + pi_proportional);

    if (output_power > pwr_set / power_step->data.int_val) {
      output_power = pwr_set / power_step->data.int_val; // Restrict integral value by upper limit
    }

    if (output_power < 0) {
      output_power = 0;
    }

    // Increment or reset power_counter variable
    if (power_counter < power_max->data.int_val / power_step->data.int_val) {
      power_counter++;
    } else {
      power_counter = 0u;
    }

    if (power_counter <= output_power && output_power) {
      heater_on = true;
      ac_load_on(heater_ac_load_num);
    } else {
      heater_on = false;
      ac_load_off(heater_ac_load_num);
    }
  }

  lcd_fb_clear(&lcd_fb);
  lcd_clear();

  std::memset(line, '\0', lcd_fb_x_size);
  std::snprintf(line, lcd_fb_x_size, "{TempHold}");
  line_ptr = &line;
  lcd_fb_print_line(&lcd_fb, current_line++, &line_ptr, lcd_align_e::LCD_ALIGN_CENTER);

  if (!std::strcmp(temp_unit, "Celsius")) {

    std::snprintf(line, sizeof(line), "%c[%.1lf/%i %cC]", temp_hold_proc_regulation_state == TEMP_HOLD_PROC_REGULATION_STATE_TEMPERATURE ? '\x7e' : ' ', kelvin_to_celsius_dbl(temperature),
                  kelvin_to_celsius(temp_set), '\xdf');
  } else if (!std::strcmp(temp_unit, "Fahrenheit")) {

    std::snprintf(line, sizeof(line), "%c[%.1lf/%i %cF]", temp_hold_proc_regulation_state == TEMP_HOLD_PROC_REGULATION_STATE_TEMPERATURE ? '\x7e' : ' ', kelvin_to_fahrenheit_dbl(temperature),
                  kelvin_to_celsius(temp_set), '\xdf');
  } else if (!std::strcmp(temp_unit, "Kelvin")) {

    std::snprintf(line, sizeof(line), "%c[%.1lf/%i %cK]", temp_hold_proc_regulation_state == TEMP_HOLD_PROC_REGULATION_STATE_TEMPERATURE ? '\x7e' : ' ', temperature, temp_set, '\xdf');
  }

  lcd_fb_print_line(&lcd_fb, current_line++, &line_ptr, lcd_align_e::LCD_ALIGN_CENTER);

  std::snprintf(line, sizeof(line), "%c[%i/%u%%]", temp_hold_proc_regulation_state == TEMP_HOLD_PROC_REGULATION_STATE_POWER ? '\x7e' : ' ', output_power * power_step->data.int_val,
                power_max->data.int_val);
  lcd_fb_print_line(&lcd_fb, current_line++, &line_ptr, lcd_align_e::LCD_ALIGN_CENTER);

  std::snprintf(line, sizeof(line), "[%02u:%02u:%02u]", rtc.hour, rtc.min, rtc.sec);
  lcd_fb_print_line(&lcd_fb, current_line++, &line_ptr, lcd_align_e::LCD_ALIGN_CENTER);

  lcd_get_window(&lcd_fb, &window, 0u, 0u, lcd_align_e::LCD_ALIGN_CENTER);
  lcd_flush_fb_window(&window);

exit:
  pi_integral = 0.0f;
  error = 0.0f;
  xSemaphoreGiveRecursive(temp_hold_proc_sem);
  return;
error:
  output_power = 0;
  pi_integral = 0.0f;
  error = 0.0f;
  ac_load_off(heater_ac_load_num);
  lcd_fb_clear(&lcd_fb);
  lcd_clear();

  std::memset(line, '\0', lcd_fb_x_size);
  std::snprintf(line, lcd_fb_x_size, "{%s}", cursor_entry_ptr->name);
  line_ptr = &line;
  lcd_fb_print_line(&lcd_fb, current_line++, &line_ptr, lcd_align_e::LCD_ALIGN_CENTER);
  std::memset(line, ' ', lcd_fb_x_size);
  lcd_fb_print_line(&lcd_fb, current_line++, &line_ptr, lcd_align_e::LCD_ALIGN_CENTER);

  if (!std::strcmp(temp_unit, "Celsius")) {

    std::snprintf(line, sizeof(line), "[???.? %cC]", '\xdf');
  } else if (!std::strcmp(temp_unit, "Fahrenheit")) {

    std::snprintf(line, sizeof(line), "[???.? %cF]", '\xdf');
  } else if (!std::strcmp(temp_unit, "Kelvin")) {

    std::snprintf(line, sizeof(line), "[???.? %cK]", '\xdf');
  }

  lcd_fb_print_line(&lcd_fb, current_line++, &line_ptr, lcd_align_e::LCD_ALIGN_CENTER);
  std::snprintf(line, sizeof(line), "[%02u:%02u:%02u, %i/%u%%]", rtc.hour, rtc.min, rtc.sec, output_power * power_step->data.int_val, power_max->data.int_val);
  lcd_fb_print_line(&lcd_fb, current_line++, &line_ptr, lcd_align_e::LCD_ALIGN_CENTER);

  lcd_get_window(&lcd_fb, &window, 0u, 0u, lcd_align_e::LCD_ALIGN_CENTER);
  lcd_flush_fb_window(&window);

  xSemaphoreGiveRecursive(temp_hold_proc_sem);
  return;
}

static void temp_hold_timer_proc_rtc_cbk(const void *data, size_t size) {
  const size_t seconds = *static_cast<const size_t *>(data);
  int32_t rc, adc_fd, rtc_fd, time, time_left;
  uint16_t adc_val;
  double temperature;
  char line[lcd_fb_x_size];
  size_t current_line = 0u, margin = 0u;
  const char(*line_ptr)[lcd_fb_x_size];
  lcd_fb_window_ta window;
  struct rtc_s rtc, rtc_time_left;
  static bool temp_reached = false;
  bool heater_on;
  static constexpr const double temp_delta = 0.5f;

  static double error;
  static double pi_integral = 0.0f;
  static double pi_proportional;
  static uint32_t power_counter = 0u;
  static int32_t output_power;

  // Get PI coefficients
  const double p_k = jfes_get_child(jfes_get_child(jfes_get_child(menu, "Settings", std::strlen("Settings")), "PI", std::strlen("PI")), "P", std::strlen("P"))->data.double_val;
  const double i_k = jfes_get_child(jfes_get_child(jfes_get_child(menu, "Settings", std::strlen("Settings")), "PI", std::strlen("PI")), "I", std::strlen("I"))->data.double_val;

  const struct jfes_value *power_max = jfes_get_child(jfes_get_child(menu_variants, "Power(%)", std::strlen("Power(%)")), "max", std::strlen("max"));
  const struct jfes_value *power_step = jfes_get_child(jfes_get_child(menu_variants, "Power(%)", std::strlen("Power(%)")), "step", std::strlen("step"));

  const struct rtc_get_time_req_s rtc_req { .rtc = &rtc };

  const char *temp_unit =
      jfes_get_child(jfes_get_child(jfes_get_child(menu, "Settings", std::strlen("Settings")), "General", std::strlen("General")), "TempUnit", std::strlen("TempUnit"))->data.string_val.data;

  const int32_t temp_set = jfes_get_child(jfes_get_child(menu, "TempHoldTimer", std::strlen("TempHoldTimer")), "Temperature", std::strlen("Temperature"))->data.int_val;
  const uint32_t time_min = jfes_get_child(jfes_get_child(menu, "TempHoldTimer", std::strlen("TempHoldTimer")), "Time(min)", std::strlen("Time(min)"))->data.int_val;
  const int32_t pwr_set = jfes_get_child(jfes_get_child(menu, "TempHoldTimer", std::strlen("TempHoldTimer")), "Power(%)", std::strlen("Power(%)"))->data.int_val;

  struct ads1118_get_temp_req_s adc_temp_req {
    .on_data_recvd = nullptr, .delay_fn = vTaskDelay, .value = &temperature
  };

  xSemaphoreTakeRecursive(temp_hold_timer_proc_sem, portMAX_DELAY);

  if (!temp_hold_timer_proc_running) {
    goto exit;
  }

  if ((rtc_fd = ::open(&sys, "rtc/rtc0", 3, 3u)) < 0) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = ::ioctl(&sys, "rtc/rtc0", rtc_ioctl_cmd_e::RTC_GET_TIME, &rtc_req, sizeof(rtc_req))) < 0) {
    if ((rtc_fd = ::close(&sys, "rtc/rtc0")) < 0) {
      brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rtc_fd = ::close(&sys, "rtc/rtc0")) < 0) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((adc_fd = ::open(&sys, "ads1118/ads1118_adc", 4, 3u)) < 0) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = ::ioctl(&sys, "ads1118/ads1118_adc", ads1118_ioctl_cmd_e::ADS1118_GET_TEMP_THERMISTOR, &adc_temp_req, sizeof(adc_temp_req))) < 0) {
    if ((rc = ::close(&sys, "ads1118/ads1118_adc")) < 0) {
      brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = ::close(&sys, "ads1118/ads1118_adc")) < 0) {
    brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  brewery_app_s::brewery_dbg_printfmt("TEMPHOLDTIMER STATE: [%02u:%02u:%02u]\r\n", rtc.hour, rtc.min, rtc.sec);

  if (temperature >= temp_set && !temp_reached) {
    rtc_reset_time();

    if ((rtc_fd = ::open(&sys, "rtc/rtc0", 3, 3u)) < 0) {
      brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    if ((rc = ::ioctl(&sys, "rtc/rtc0", rtc_ioctl_cmd_e::RTC_GET_TIME, &rtc_req, sizeof(rtc_req))) < 0) {
      if ((rtc_fd = ::close(&sys, "rtc/rtc0")) < 0) {
        brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
        goto error;
      }

      brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    if ((rtc_fd = ::close(&sys, "rtc/rtc0")) < 0) {
      brewery_app_s::brewery_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    temp_reached = true;
  }

  time = time_min * 60u;
  time_left = time;

  if (temp_reached) {
    time_left = time - (rtc.sec + rtc.min * 60u + rtc.hour * 3600u);
  }

  rtc_time_left.sec = time_left % 60u;
  rtc_time_left.min = (time_left / 60u) % 60u;
  rtc_time_left.hour = (time_left / 60u) / 60u;

  // Calculate PI
  if (temp_set) {
    error = temp_set - temperature;
    pi_integral += error * i_k;    // PI integral part
    pi_proportional = error * p_k; // PI proportional part

    for (double *const pid_parameter : {&pi_proportional, &pi_integral}) {
      if (*pid_parameter > static_cast<double>(power_max->data.int_val / power_step->data.int_val)) {
        *pid_parameter = static_cast<double>(power_max->data.int_val / power_step->data.int_val); // Restrict integral value by upper limit
      }
    }

    output_power = static_cast<uint32_t>(pi_integral + pi_proportional);

    if (output_power > pwr_set / power_step->data.int_val) {
      output_power = pwr_set / power_step->data.int_val; // Restrict output by upper limit
    }

    if (output_power < 0) {
      output_power = 0;
    }

    // Increment or reset power_counter variable
    if (power_counter < power_max->data.int_val / power_step->data.int_val) {
      power_counter++;
    } else {
      power_counter = 0u;
    }

    if (power_counter <= output_power && output_power) {
      heater_on = true;
      ac_load_on(heater_ac_load_num);
    } else {
      heater_on = false;
      ac_load_off(heater_ac_load_num);
    }
  }

  lcd_fb_clear(&lcd_fb);
  lcd_clear();

  std::memset(line, '\0', lcd_fb_x_size);
  std::snprintf(line, lcd_fb_x_size, "{TempHoldTimer}");
  line_ptr = &line;
  lcd_fb_print_line(&lcd_fb, current_line++, &line_ptr, lcd_align_e::LCD_ALIGN_CENTER);

  if (!std::strcmp(temp_unit, "Celsius")) {

    std::snprintf(line, sizeof(line), "%c[%.1lf/%i %cC]", temp_hold_proc_regulation_state == TEMP_HOLD_PROC_REGULATION_STATE_TEMPERATURE ? '\x7e' : ' ', kelvin_to_celsius_dbl(temperature),
                  kelvin_to_celsius(temp_set), '\xdf');
  } else if (!std::strcmp(temp_unit, "Fahrenheit")) {

    std::snprintf(line, sizeof(line), "%c[%.1lf/%i %cF]", temp_hold_proc_regulation_state == TEMP_HOLD_PROC_REGULATION_STATE_TEMPERATURE ? '\x7e' : ' ', kelvin_to_fahrenheit_dbl(temperature),
                  kelvin_to_celsius(temp_set), '\xdf');
  } else if (!std::strcmp(temp_unit, "Kelvin")) {

    std::snprintf(line, sizeof(line), "%c[%.1lf/%i %cK]", temp_hold_proc_regulation_state == TEMP_HOLD_PROC_REGULATION_STATE_TEMPERATURE ? '\x7e' : ' ', temperature, temp_set, '\xdf');
  }

  lcd_fb_print_line(&lcd_fb, current_line++, &line_ptr, lcd_align_e::LCD_ALIGN_CENTER);

  std::snprintf(line, sizeof(line), "%c[%i/%u%%]", temp_hold_proc_regulation_state == TEMP_HOLD_PROC_REGULATION_STATE_POWER ? '\x7e' : ' ', output_power * power_step->data.int_val,
                power_max->data.int_val);
  lcd_fb_print_line(&lcd_fb, current_line++, &line_ptr, lcd_align_e::LCD_ALIGN_CENTER);

  std::snprintf(line, sizeof(line), "[%02u:%02u:%02u]", (temp_reached ? rtc_time_left : rtc).hour, (temp_reached ? rtc_time_left : rtc).min, (temp_reached ? rtc_time_left : rtc).sec);
  lcd_fb_print_line(&lcd_fb, current_line++, &line_ptr, lcd_align_e::LCD_ALIGN_CENTER);

  lcd_get_window(&lcd_fb, &window, 0u, 0u, lcd_align_e::LCD_ALIGN_CENTER);
  lcd_flush_fb_window(&window);

  if (!time_left) {
    temp_hold_timer_proc_stop();
    rtc_reset_time();
    temp_reached = false;
    trigger_json_val(&lcd_fb, cursor_entry_ptr->parent, cursor_entry_ptr->parent->name ? cursor_entry_ptr->parent->name : application_name);
    state = MENU_MODE_ENTRIES_SURFING;
  }

exit:
  pi_integral = 0.0f;
  error = 0.0f;
  xSemaphoreGiveRecursive(temp_hold_timer_proc_sem);
  return;
error:
  output_power = 0;
  pi_integral = 0.0f;
  error = 0.0f;
  ac_load_off(heater_ac_load_num);
  lcd_fb_clear(&lcd_fb);
  lcd_clear();

  std::memset(line, '\0', lcd_fb_x_size);
  std::snprintf(line, lcd_fb_x_size, "{%s}", cursor_entry_ptr->name);
  line_ptr = &line;
  lcd_fb_print_line(&lcd_fb, current_line++, &line_ptr, lcd_align_e::LCD_ALIGN_CENTER);
  std::memset(line, ' ', lcd_fb_x_size);
  lcd_fb_print_line(&lcd_fb, current_line++, &line_ptr, lcd_align_e::LCD_ALIGN_CENTER);

  if (!std::strcmp(temp_unit, "Celsius")) {

    std::snprintf(line, sizeof(line), "[???.? %cC]", '\xdf');
  } else if (!std::strcmp(temp_unit, "Fahrenheit")) {

    std::snprintf(line, sizeof(line), "[???.? %cF]", '\xdf');
  } else if (!std::strcmp(temp_unit, "Kelvin")) {

    std::snprintf(line, sizeof(line), "[???.? %cK]", '\xdf');
  }

  lcd_fb_print_line(&lcd_fb, current_line++, &line_ptr, lcd_align_e::LCD_ALIGN_CENTER);
  std::snprintf(line, sizeof(line), "[%02u:%02u:%02u, %i/%u%%]", rtc.hour, rtc.min, rtc.sec, output_power * power_step->data.int_val, power_max->data.int_val);
  lcd_fb_print_line(&lcd_fb, current_line++, &line_ptr, lcd_align_e::LCD_ALIGN_CENTER);

  lcd_get_window(&lcd_fb, &window, 0u, 0u, lcd_align_e::LCD_ALIGN_CENTER);
  lcd_flush_fb_window(&window);

  xSemaphoreGiveRecursive(temp_hold_timer_proc_sem);
  return;
}

static void rtc_callback(const void *data, size_t size) {
  if (thermometer_proc_running) {
    thermometer_proc_rtc_cbk(data, size);
  }

  if (temp_hold_proc_running) {
    temp_hold_proc_rtc_cbk(data, size);
  }

  if (temp_hold_timer_proc_running) {
    temp_hold_timer_proc_rtc_cbk(data, size);
  }

  if (brewery_proc_running) {
    brewery_proc_rtc_cbk(data, size);
  }
}

static void beep() {
  int32_t rc, fd;
  struct buzzer_beep_req_s req {
    .delay_fn = vTaskDelay, .up = 250u, .down = 0u, .n = 1u
  };

  if ((fd = ::open(&sys, "buzzer/buzzer0", 3, 3u)) < 0) {
    goto error;
  }

  if ((rc = ::ioctl(&sys, "buzzer/buzzer0", buzzer_drv_ioctl_cmd_e::BUZZER_BEEP, &req, sizeof(req))) < 0) {
    goto error;
  }

  if ((rc = ::close(&sys, "buzzer/buzzer0")) < 0) {
    goto error;
  }

exit:
  return;
error:
  return;
}

static void beep_short() {
  int32_t rc, fd;
  struct buzzer_beep_req_s req {
    .delay_fn = vTaskDelay, .up = 50u, .down = 0u, .n = 1u
  };

  if ((fd = ::open(&sys, "buzzer/buzzer0", 3, 3u)) < 0) {
    goto error;
  }

  if ((rc = ::ioctl(&sys, "buzzer/buzzer0", buzzer_drv_ioctl_cmd_e::BUZZER_BEEP, &req, sizeof(req))) < 0) {
    goto error;
  }

  if ((rc = ::close(&sys, "buzzer/buzzer0")) < 0) {
    goto error;
  }

exit:
  return;
error:
  return;
}
