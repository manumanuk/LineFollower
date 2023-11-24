#ifndef TCS_3472_H_
#define TCS_3472_H_

#include <stdint.h>
#include <stdbool.h>

#define TCS_I2C_ADDR (0x29 << 1)
#define TCS_I2C_TIMEOUT_MS 1000

#define TCS_CMD_REG_BIT (1U << 7)
#define TCS_CMD_TYPE_INCREMENT (1U << 4)
#define TCS_CMD_TYPE_SPECIAL_FUNC (2U << 5)
#define TCS_CMD_SF_CLR_CH_INT (0b00110U)

#define TCS_REG_ENABLE 0x00U
#define TCS_REG_ENABLE_MASK_AIEN (1U << 4)
#define TCS_REG_ENABLE_MASK_WEN (1U << 3)
#define TCS_REG_ENABLE_MASK_AEN (1U << 1)
#define TCS_REG_ENABLE_MASK_PON 1U
#define TCS_REG_ATIME 0x01U
#define TCS_REG_WTIME 0x03U
#define TCS_REG_AILTL 0x04U
#define TCS_REG_AILTH 0x05U
#define TCS_REG_AIHTL 0x06U
#define TCS_REG_AIHTH 0x07U
#define TCS_REG_PERS 0x0CU
#define TCS_REG_CONFIG 0x0DU
#define TCS_REG_MASK_CONFIG_WLONG (1U << 1)
#define TCS_REG_CONTROL 0x0FU
#define TCS_REG_ID 0x12U
#define TCS_REG_STATUS 0x13U
#define TCS_REG_STATUS_MASK_AVALID 1U
#define TCS_REG_CDATA 0x14U
#define TCS_REG_CDATAH 0x15U
#define TCS_REG_RDATA 0x16U
#define TCS_REG_RDATAH 0x17U
#define TCS_REG_GDATA 0x18U
#define TCS_REG_GDATAH 0x19U
#define TCS_REG_BDATA 0x1AU
#define TCS_REG_BDATAH 0x1BU

#define TCS_TOTAL_DATA_BYTES 8U

#define LEFT_RED_R 0.55
#define LEFT_BLUE_B 0.45
#define LEFT_BROWN_R 0.33
// #define LEFT_GREEN_G 0.45

#define RIGHT_RED_R 0.56
#define RIGHT_BLUE_B 0.42
#define RIGHT_BROWN_R 0.38
// #define RIGHT_GREEN_G 0.45

typedef enum {
    LEFT_COLOUR_SENSOR,
    RIGHT_COLOUR_SENSOR
} tcs3472_instance_e;

void tcs3472_init(tcs3472_instance_e instance);
bool tcs3472_get_colour_data(tcs3472_instance_e instance, float *rgb);

#endif /* TCS_3472_H_ */
