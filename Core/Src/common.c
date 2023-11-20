#include <stdio.h>
#include <stdint.h>
#include "common.h"
#include "stm32f4xx_hal.h"

#define MAX_BUF_SIZE 500U

extern UART_HandleTypeDef huart2;

void serial_print(char* s, ...) {
  char buf[MAX_BUF_SIZE] = {0};

  va_list args;
  va_start(args, s);
  uint16_t n = vsnprintf(buf, MAX_BUF_SIZE, s, args);
  HAL_UART_Transmit(&huart2, buf, n, HAL_MAX_DELAY);
  va_end(args);
}
