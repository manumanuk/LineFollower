#include "tcs3472.h"
#include "stm32f4xx_hal.h"

extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c3;
extern void Error_Handler();

static void tcs3472_convert_raw_to_normalized(uint16_t *rgbc, float *colourData);

typedef struct {
  uint8_t addr;
  uint8_t val;
} tcs_settings_t;

static tcs_settings_t settings[] = {
  {TCS_REG_ENABLE, TCS_REG_ENABLE_MASK_AEN | TCS_REG_ENABLE_MASK_PON},
  {TCS_REG_ATIME, 0xFFU},
  {TCS_REG_CONTROL, 1U}
};

void tcs3472_init(tcs3472_instance_e instance) {
  I2C_HandleTypeDef *i2cBusPtr = (instance == LEFT_COLOUR_SENSOR) ? &hi2c1 : &hi2c3;
  for (uint8_t i=0; i<sizeof(settings)/sizeof(tcs_settings_t); i++) {
    uint8_t wData[2] = {TCS_CMD_REG_BIT | settings[i].addr,
                        settings[i].val};
    
    if (HAL_I2C_Master_Transmit(i2cBusPtr, TCS_I2C_ADDR, wData, 2, TCS_I2C_TIMEOUT_MS) != HAL_OK)
      Error_Handler();
  }
}

bool tcs3472_get_colour_data(tcs3472_instance_e instance, float *colourData) {
  I2C_HandleTypeDef *i2cBusPtr = (instance == LEFT_COLOUR_SENSOR) ? &hi2c1 : &hi2c3;

  uint8_t wData[] = {TCS_CMD_REG_BIT | TCS_CMD_TYPE_INCREMENT | TCS_REG_STATUS};
  if (HAL_I2C_Master_Transmit(i2cBusPtr, TCS_I2C_ADDR, wData, sizeof(wData), TCS_I2C_TIMEOUT_MS) != HAL_OK)
    Error_Handler();

  uint8_t rData[TCS_TOTAL_DATA_BYTES+1];
  if (HAL_I2C_Master_Receive(i2cBusPtr, TCS_I2C_ADDR, rData, TCS_TOTAL_DATA_BYTES+1, TCS_I2C_TIMEOUT_MS) != HAL_OK)
    Error_Handler();
  
  if (((rData[0]) & TCS_REG_STATUS_MASK_AVALID) == 0) {
    return false;
  }

  uint16_t rgbc[4];
  for (uint8_t i=0; i<4; i++) {
    rgbc[i] = ((uint16_t)rData[1+i*2]) | (((uint16_t)rData[1+i*2+1]) << 8U);
  }

  tcs3472_convert_raw_to_normalized(rgbc, colourData);

  return true;
}

static void tcs3472_convert_raw_to_normalized(uint16_t *rgbc, float *colourData) {
  colourData[0] = ((float)rgbc[1])/rgbc[0];
  colourData[1] = ((float)rgbc[2])/rgbc[0];
  colourData[2] = ((float)rgbc[3])/rgbc[0];
}
