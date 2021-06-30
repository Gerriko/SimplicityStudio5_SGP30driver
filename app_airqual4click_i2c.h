/***************************************************************************//**
 * @file
 * @brief Air Quality 4 click i2c functions
 *******************************************************************************
 * # MIT License
 * <b>Copyright 2021 Colin Gerrish</b>
 ******************************************************************************/


#ifndef APP_AIRQUAL4CLICK_I2C_H
#define APP_AIRQUAL4CLICK_I2C_H

#include <stdio.h>
#include <string.h>
#include "em_chip.h"
#include "sl_status.h"
#include "sl_simple_led_instances.h"

#include "sl_sleeptimer.h"


/***************************************************************************//**
 * Initialize air quality 4 click i2c
 ******************************************************************************/
void app_airqual4click_i2c_init(void);


/***************************************************************************//**
 * Retrieves CO2 and VOC values from air quality 4 click i2c
 ******************************************************************************/
bool get_airqualitydata(uint16_t* CO2, uint16_t* TVOC);

#endif  // APP_AIRQUAL4CLICK_I2C_H
