#include "tcs3472.h"
#include "stm32f4xx_hal.h"

extern I2C_HandleTypeDef hi2c1;
extern void Error_Handler();

typedef struct {
  uint8_t addr;
  uint8_t val;
} tcs_settings_t;

static tcs_settings_t settings[] = {
  {TCS_REG_ENABLE, TCS_REG_ENABLE_MASK_AEN | TCS_REG_ENABLE_MASK_PON},
  {TCS_REG_ATIME, 0xFFU},
  {TCS_REG_CONTROL, 1U}
};

void tcs3472_init(void) {
  for (uint8_t i=0; i<sizeof(settings)/sizeof(tcs_settings_t); i++) {
    uint8_t wData[2] = {TCS_CMD_REG_BIT | settings[i].addr,
                        settings[i].val};
    
    if (HAL_I2C_Master_Transmit(&hi2c1, TCS_I2C_ADDR, wData, 2, TCS_I2C_TIMEOUT_MS) != HAL_OK)
      Error_Handler();
  }
}

bool tcs3472_get_colour_data(uint16_t *rgbc) {
  uint8_t wData[] = {TCS_CMD_REG_BIT | TCS_CMD_TYPE_INCREMENT | TCS_REG_STATUS};
  if (HAL_I2C_Master_Transmit(&hi2c1, TCS_I2C_ADDR, wData, sizeof(wData), TCS_I2C_TIMEOUT_MS) != HAL_OK)
    Error_Handler();

  uint8_t rData[TCS_TOTAL_DATA_BYTES+1];
  if (HAL_I2C_Master_Receive(&hi2c1, TCS_I2C_ADDR, rData, TCS_TOTAL_DATA_BYTES+1, TCS_I2C_TIMEOUT_MS) != HAL_OK)
    Error_Handler();
  
  if (((rData[0]) & TCS_REG_STATUS_MASK_AVALID) == 0) {
    return false;
  }

  for (uint8_t i=0; i<4; i++) {
    rgbc[i] = ((uint16_t)rData[1+i*2]) | (((uint16_t)rData[1+i*2+1]) << 8U);
  }

  return true;
}

static void tcs3472_convert_raw_to_normalized(uint16_t *rgbc, double *colourData) {
  colourData[0] = ((double)rgbc[1])/rgbc[0];
  colourData[1] = ((double)rgbc[2])/rgbc[0];
  colourData[2] = ((double)rgbc[3])/rgbc[0];
}

bool check_green() {
    uint16_t rgbc[4];
    double colourData[3];
    if(tcs3472_get_colour_data(rgbc)) {
        tcs3472_convert_raw_to_normalized(rgbc, colourData);
        if (colourData[0] < 0.26 && colourData[1] > 0.46) {
            return true;
        }
    }

    return false;
}

bool check_blue() {
    uint16_t rgbc[4];
    double colourData[3];
    if(tcs3472_get_colour_data(rgbc)) {
        tcs3472_convert_raw_to_normalized(rgbc, colourData);
        if (colourData[0] > 0.45) {
            return true;
        }
    }

    return false;
}

