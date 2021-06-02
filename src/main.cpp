#include "sys/system.hpp"

#include "boards/board_tags.hpp"

#include "drivers/ac_load_driver.hpp"
#include "drivers/ads1118_driver.hpp"
#include "drivers/button_driver.hpp"
#include "drivers/buzzer_driver.hpp"
#include "drivers/gpio_driver.hpp"
#include "drivers/hd44780_driver.hpp"
#include "drivers/leds_driver.hpp"
#include "drivers/rcc_driver.hpp"
#include "drivers/relay_load_driver.hpp"
#include "drivers/rtc_driver.hpp"
#include "drivers/spi_driver.hpp"
#include "drivers/timer_driver.hpp"
#include "drivers/usart_driver.hpp"
#include "drivers/w25qxx_driver.hpp"

#include "tasks/events_worker.hpp"
#include "tasks/init_task.hpp"

#include "apps/bender/bender.hpp"
#include "apps/brewery/brewery.hpp"
#include "apps/clear/clear.hpp"
#include "apps/cowsay/cowsay.hpp"
#include "apps/debug/debug.hpp"
#include "apps/reboot/reboot.hpp"
#include "apps/shell/shell.hpp"

// Global flags
bool debug_log_enabled = false;

/* Init & get references to modules */
auto &rcc = make_drv<struct rcc_drv_name_s>(&rcc_drv_ops);
auto &gpio = make_drv<1u, struct gpio_drv_name_s>(&gpio_drv_ops, {&rcc});
auto &usart = make_drv<2u, struct usart_drv_name_s>(&usart_drv_ops, {&rcc, &gpio});
auto &rtc = make_drv<2u, struct rtc_drv_name_s>(&rtc_drv_ops, {&rcc, &usart});
auto &buzzer = make_drv<3u, struct buzzer_drv_name_s>(&buzzer_drv_ops, {&rcc, &gpio, &usart});
auto &tim = make_drv<3u, struct timer_drv_name_s>(&timer_drv_ops, {&rcc, &gpio, &usart});
auto &spi = make_drv<3u, struct spi_drv_name_s>(&spi_drv_ops, {&rcc, &gpio, &usart});
auto &ac_load = make_drv<2u, struct ac_load_drv_name_s>(&ac_load_drv_ops, {&gpio, &usart});
auto &relay_load = make_drv<2u, struct relay_load_drv_name_s>(&relay_load_drv_ops, {&gpio, &usart});
auto &buttons = make_drv<2u, struct button_drv_name_s>(&button_drv_ops, {&gpio, &usart});
auto &leds = make_drv<2u, struct leds_drv_name_s>(&leds_drv_ops, {&gpio, &usart});
auto &hd44780 = make_drv<2u, struct hd44780_drv_name_s>(&hd44780_drv_ops, {&gpio, &usart});
auto &w25qxx = make_drv<3u, struct w25qxx_drv_name_s>(&w25qxx_drv_ops, {&gpio, &spi, &usart});
auto &ads1118 = make_drv<3u, struct ads1118_drv_name_s>(&ads1118_drv_ops, {&gpio, &spi, &usart});
auto &ic74hc595 = make_drv<2u, struct ic74hc595_drv_name_s>(&ic74hc595_drv_ops, {&gpio, &usart});
auto &uln2003 = make_drv<2u, struct uln2003_drv_name_s>(&uln2003_drv_ops, {&gpio, &usart});
auto &zsp3806g2e = make_drv<3u, struct zsp3806g2e_drv_name_s>(&zsp3806g2e_drv_ops, {&gpio, &usart, &spi});
auto &cl5621ah = make_drv<3u, struct cl5621ah_drv_name_s>(&cl5621ah_drv_ops, {&gpio, &usart, &ic74hc595});

// Module array
const struct drv_model_cmn_s *modules[]{&rcc,     &gpio, &usart,   &rtc,    &buzzer,  &tim,       &spi,     &ac_load,    &relay_load,
                                        &buttons, &leds, &hd44780, &w25qxx, &ads1118, &ic74hc595, &uln2003, &zsp3806g2e, &cl5621ah};

// Applications
struct app_s shell_app = {.name = "shell", .entry = shell_app_s::entry};
struct app_s clear_app = {.name = "clear", .entry = clear_app_s::entry};
struct app_s cowsay_app = {.name = "cowsay", .entry = cowsay_app_s::entry};
struct app_s debug_app = {.name = "debug", .entry = debug_app_s::entry};
struct app_s reboot_app = {.name = "reboot", .entry = reboot_app_s::entry};
struct app_s brewery_app = {.name = "brewery", .entry = brewery_app_s::entry};
struct app_s brewery_app = {.name = "bender", .entry = bender_app_s::entry};

// Application array
const struct app_s *apps[]{&shell_app, &clear_app, &cowsay_app, &debug_app, &reboot_app, &brewery_app, &bender_app};

/* Create System object and get reference */
auto &sys = make_system(modules, sizeof(modules) / sizeof(modules[0u]), apps, sizeof(apps) / sizeof(apps[0u]));

void init() {

  // Events worker
  static auto &ew = make_ew();
  static xTaskHandle init_task_handle, events_worker_task_handle;

  // Init events worker
  ew.init();
 
  // Run events worker task
  xTaskCreate(events_worker_task_code, "events_worker_task", configMINIMAL_STACK_SIZE * 24u, &ew, 10u, &events_worker_task_handle);

  // Run init task
  xTaskCreate(init_task_code, "init_task", configMINIMAL_STACK_SIZE * 6u, nullptr, 5u, &init_task_handle);
}

int main(int argc, char *argv[]) {
  // Init and run scheduler
  sys.run(init, vTaskStartScheduler);

  // Shalln't reach here
  return -1;
}
