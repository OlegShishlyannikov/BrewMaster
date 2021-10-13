#include "boards/board_tags.hpp"
#include "sys/system.hpp"

#if defined(BREWERY_CONTROLLER_APP)

#  include "drivers/ac_load_driver.hpp"
#  include "drivers/ads1118_driver.hpp"
#  include "drivers/button_driver.hpp"
#  include "drivers/buzzer_driver.hpp"
#  include "drivers/gpio_driver.hpp"
#  include "drivers/hd44780_driver.hpp"
#  include "drivers/leds_driver.hpp"
#  include "drivers/rcc_driver.hpp"
#  include "drivers/relay_load_driver.hpp"
#  include "drivers/rtc_driver.hpp"
#  include "drivers/spi_driver.hpp"
#  include "drivers/timer_driver.hpp"
#  include "drivers/usart_driver.hpp"
#  include "drivers/w25qxx_driver.hpp"

#  include "tasks/events_worker.hpp"
#  include "tasks/init_task.hpp"

#  include "apps/brewery/brewery.hpp"
#  include "apps/clear/clear.hpp"
#  include "apps/cowsay/cowsay.hpp"
#  include "apps/debug/debug.hpp"
#  include "apps/reboot/reboot.hpp"
#  include "apps/shell/shell.hpp"

#elif defined(BENDER_CONTROLLER_APP)

#  include "drivers/74hc595_driver.hpp"
#  include "drivers/ac_load_driver.hpp"
#  include "drivers/buzzer_driver.hpp"
#  include "drivers/gpio_driver.hpp"
#  include "drivers/leds_driver.hpp"
#  include "drivers/modbus_driver.hpp"
#  include "drivers/vfd_driver.hpp"
#  include "drivers/rcc_driver.hpp"
#  include "drivers/relay_load_driver.hpp"
#  include "drivers/rtc_driver.hpp"
#  include "drivers/spi_driver.hpp"
#  include "drivers/timer_driver.hpp"
#  include "drivers/ui_driver.hpp"
#  include "drivers/uln2003_driver.hpp"
#  include "drivers/usart_driver.hpp"
#  include "drivers/zsp3806g2e_driver.hpp"

#  include "tasks/events_worker.hpp"
#  include "tasks/init_task.hpp"

#  include "apps/bender/bender.hpp"
#  include "apps/clear/clear.hpp"
#  include "apps/cowsay/cowsay.hpp"
#  include "apps/debug/debug.hpp"
#  include "apps/modbus/modbus.hpp"
#  include "apps/reboot/reboot.hpp"
#  include "apps/shell/shell.hpp"

#endif

// Global flags
bool debug_log_enabled = false;

#if defined(BREWERY_CONTROLLER_APP)

/* Init & get references to modules */
static auto &rcc = make_drv<struct rcc_drv_name_s>(&rcc_drv_ops);
static auto &gpio = make_drv<1u, struct gpio_drv_name_s>(&gpio_drv_ops, {&rcc});
static auto &usart = make_drv<2u, struct usart_drv_name_s>(&usart_drv_ops, {&rcc, &gpio});
static auto &rtc = make_drv<2u, struct rtc_drv_name_s>(&rtc_drv_ops, {&rcc, &usart});
static auto &buzzer = make_drv<3u, struct buzzer_drv_name_s>(&buzzer_drv_ops, {&rcc, &gpio, &usart});
static auto &tim = make_drv<3u, struct timer_drv_name_s>(&timer_drv_ops, {&rcc, &gpio, &usart});
static auto &spi = make_drv<3u, struct spi_drv_name_s>(&spi_drv_ops, {&rcc, &gpio, &usart});
static auto &ac_load = make_drv<2u, struct ac_load_drv_name_s>(&ac_load_drv_ops, {&gpio, &usart});
static auto &relay_load = make_drv<2u, struct relay_load_drv_name_s>(&relay_load_drv_ops, {&gpio, &usart});
static auto &leds = make_drv<2u, struct leds_drv_name_s>(&leds_drv_ops, {&gpio, &usart});
static auto &hd44780 = make_drv<2u, struct hd44780_drv_name_s>(&hd44780_drv_ops, {&gpio, &usart});
static auto &w25qxx = make_drv<3u, struct w25qxx_drv_name_s>(&w25qxx_drv_ops, {&gpio, &spi, &usart});
static auto &ads1118 = make_drv<3u, struct ads1118_drv_name_s>(&ads1118_drv_ops, {&gpio, &spi, &usart});
static auto &buttons = make_drv<3u, struct button_drv_name_s>(&button_drv_ops, {&gpio, &usart, &uln2003});

// Module array
static const struct drv_model_cmn_s *modules[]{&rcc, &gpio, &usart, &rtc, &buzzer, &tim, &spi, &ac_load, &relay_load, &leds, &hd44780, &w25qxx, &ads1118};

// Applications
static struct app_s shell_app = {.name = "shell", .entry = shell_app_s::entry};
static struct app_s clear_app = {.name = "clear", .entry = clear_app_s::entry};
static struct app_s cowsay_app = {.name = "cowsay", .entry = cowsay_app_s::entry};
static struct app_s debug_app = {.name = "debug", .entry = debug_app_s::entry};
static struct app_s reboot_app = {.name = "reboot", .entry = reboot_app_s::entry};
static struct app_s bender_app = {.name = "bender", .entry = bender_app_s::entry};

// Application array
const struct app_s *apps[]{&shell_app, &clear_app, &cowsay_app, &debug_app, &reboot_app, &bender_app};

#elif defined(BENDER_CONTROLLER_APP)

/* Init & get references to modules */
static auto &rcc = make_drv<struct rcc_drv_name_s>(&rcc_drv_ops);
static auto &gpio = make_drv<1u, struct gpio_drv_name_s>(&gpio_drv_ops, {&rcc});
static auto &usart = make_drv<2u, struct usart_drv_name_s>(&usart_drv_ops, {&rcc, &gpio});
static auto &modbus = make_drv<1u, struct modbus_drv_name_s>(&modbus_drv_ops, {&usart});
static auto &vfd = make_drv<1u, struct vfd_drv_name_s>(&vfd_drv_ops, {&modbus});
static auto &rtc = make_drv<2u, struct rtc_drv_name_s>(&rtc_drv_ops, {&rcc, &usart});
static auto &buzzer = make_drv<3u, struct buzzer_drv_name_s>(&buzzer_drv_ops, {&rcc, &gpio, &usart});
static auto &tim = make_drv<3u, struct timer_drv_name_s>(&timer_drv_ops, {&rcc, &gpio, &usart});
static auto &spi = make_drv<3u, struct spi_drv_name_s>(&spi_drv_ops, {&rcc, &gpio, &usart});
static auto &relay_load = make_drv<2u, struct relay_load_drv_name_s>(&relay_load_drv_ops, {&gpio, &usart});
static auto &leds = make_drv<2u, struct leds_drv_name_s>(&leds_drv_ops, {&gpio, &usart});
static auto &uln2003 = make_drv<3u, struct uln2003_drv_name_s>(&uln2003_drv_ops, {&gpio, &usart, &tim});
static auto &ic74hc595 = make_drv<3u, struct ic74hc595_drv_name_s>(&ic74hc595_drv_ops, {&gpio, &usart, &spi});
static auto &zsp3806g2e = make_drv<3u, struct zsp3806g2e_drv_name_s>(&zsp3806g2e_drv_ops, {&gpio, &usart, &spi});
static auto &ui = make_drv<7u, struct ui_drv_name_s>(&ui_drv_ops, {&gpio, &usart, &ic74hc595, &uln2003, &tim, &buzzer, &zsp3806g2e});

// Module array
static const struct drv_model_cmn_s *modules[] = {&rcc, &gpio, &usart, &rtc, &buzzer, &tim, &spi, &relay_load, &leds,
                                           &ic74hc595, &uln2003, &zsp3806g2e, &ui,
                                           &modbus, &vfd};

// Applications
static struct app_s shell_app = {.name = "shell", .entry = shell_app_s::entry};
static struct app_s clear_app = {.name = "clear", .entry = clear_app_s::entry};
static struct app_s cowsay_app = {.name = "cowsay", .entry = cowsay_app_s::entry};
static struct app_s debug_app = {.name = "debug", .entry = debug_app_s::entry};
static struct app_s reboot_app = {.name = "reboot", .entry = reboot_app_s::entry};
static struct app_s modbus_test_app = {.name = "modbus", .entry = modbus_app_s::entry};
static struct app_s bender_app = {.name = "bender", .entry = bender_app_s::entry};

// Application array
const struct app_s *apps[]{
    &shell_app,
    &clear_app, &cowsay_app, &debug_app, &reboot_app,
	  &modbus_test_app,
    &bender_app
};

#endif

/* Create System object and get reference */
auto &sys = make_system(modules, sizeof(modules) / sizeof(modules[0u]), apps, sizeof(apps) / sizeof(apps[0u]));
const char *shell_cmdline_args = "nothing here!";

void init() {
  
  // Events worker
  static auto &ew = make_ew();
  static xTaskHandle init_task_handle, events_worker_task_handle;

  // Init events worker
  ew.init();

  // Run events worker task
  xTaskCreate(events_worker_task_code, "events_worker_task", configMINIMAL_STACK_SIZE * 24u, &ew, 10u, &events_worker_task_handle);

  // Run init task
  xTaskCreate(init_task_code, "init_task", configMINIMAL_STACK_SIZE * 6u, &shell_cmdline_args, 5u, &init_task_handle);
}

int main(int argc, char *argv[]) {
  // Init and run scheduler
  sys.run(init, vTaskStartScheduler);

  // Shalln't reach here
  return -1;
}
