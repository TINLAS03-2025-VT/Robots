#ifndef __LOG_H__
#define __LOG_H__

#define KRED "\x1B[31m"
#define KGRN "\x1B[32m"
#define KYEL "\x1B[33m"
#define KNRM "\x1B[0m" // Reset to normal color

// Prints a green success message
#define PRINT_SUCCESS(format, ...)                                             \
  printf("%s[SUCCESS] " format "%s\n", KGRN, ##__VA_ARGS__, KNRM)

// Prints a yellow debug message
#define PRINT_DEBUG(format, ...)                                               \
  printf("%s[DEBUG] " format "%s\n", KYEL, ##__VA_ARGS__, KNRM)

// SOFTCHECK: Logs the critical error, turns off the LED, but lets the program
// continue
#define SOFTCHECK(format, ...)                                                 \
  printf("%s[CRITICAL ERROR] " format "%s\n", KRED, ##__VA_ARGS__, KNRM);      \
  cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);

// HARDCHECK: Logs the critical error, turns off the LED, waits 2 seconds, and
// reboots via watchdog
#define HARDCHECK(format, ...)                                                 \
  printf("%s[CRITICAL ERROR] " format                                          \
         ". Resetting system in 2 seconds...%s\n",                             \
         KRED, ##__VA_ARGS__, KNRM);                                           \
  cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);                               \
  sleep_ms(2000);                                                              \
  watchdog_reboot(0, 0, 0);

#endif