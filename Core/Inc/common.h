#ifndef COMMON_H_
#define COMMON_H_

#include <stdarg.h>

#define DEBUG_MODE 1

#define MAX_IR_ARRAY_SENSORS 8U
#define IR_ARRAY_ADC_TIMEOUT 400U
#define FRONT_IR_ARRAY_SENSORS 8U

#if DEBUG_MODE
#define BACK_IR_ARRAY_SENSORS 6U
#else
#define BACK_IR_ARRAY_SENSORS 8U
#endif

#define SW_PORT GPIOC
#define SW_PIN GPIO_PIN_13

#define LD2_Pin GPIO_PIN_2
#define LD2_GPIO_Port GPIOB

#define max(a, b) (((a) > (b)) ? (a) : (b))

typedef enum {
	RED = 0,
	BROWN,
	GREEN,
	BLUE
} line_colour_e;

void serial_print(char* format, ...);

#endif /* COMMON_H_ */
