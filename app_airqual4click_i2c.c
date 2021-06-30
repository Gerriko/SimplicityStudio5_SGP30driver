/***************************************************************************//**
 * @file
 * @brief iostream usart examples functions
 *******************************************************************************
 * # License
 * <b>Copyright 2020 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/

#include "app_airqual4click_i2c.h"
#include "sgp30.h"
#include "app_dht11_gpio.h"

/*******************************************************************************
 *******************************   DEFINES   ***********************************
 ******************************************************************************/
#ifndef SENSORMEASURE_TICK
#define SENSORMEASURE_TICK         (1000u)
#endif


/*******************************************************************************
 ***************************  LOCAL VARIABLES   ********************************
 ******************************************************************************/
bool allgood = false;
bool airqual4Init = false;
bool SGP30warmup = false;
bool SensorStop = false;

sl_sleeptimer_timer_handle_t SensorMeasure_timer;
volatile bool toggle_timeout = false;
volatile uint32_t toggle_counter = 0;


/*******************************************************************************
 *********************   LOCAL FUNCTION PROTOTYPES   ***************************
 ******************************************************************************/

static void on_timer_callback(sl_sleeptimer_timer_handle_t *handle,
                       void *data);

/*******************************************************************************
 **************************   GLOBAL FUNCTIONS   *******************************
 ******************************************************************************/


/***************************************************************************//**
 * Initialise air quality 4 click.
 ******************************************************************************/
void app_airqual4click_i2c_init(void)
{
  //sl_emlib_gpio_simple_init();
  sl_simple_led_init_instances();
  sl_i2cspm_init_instances();

  app_dht11_gpio_init();      // initialise the dht11 sensor

  if (!initAirQuality()) {
      airqual4Init = true;
      printf("AIR QUALITY 4 CLICK started\r\n");
  }
  else printf("AIR QUALITY 4 CLICK failed to start :-(\r\n");

}

bool get_airqualitydata(uint16_t* CO2, uint16_t* TVOC)
{
  uint16_t _CO2 = 0;
  uint16_t _TVOC = 0;
  uint16_t _TestDHT11 = 0;

  allgood = false;
  if (airqual4Init) {
      printf("Measuring Air Quality...\r\n");

      sl_status_t status;

      if (!SGP30warmup) printf(" ...sorry, need warm up first (approx. 15 secs)\r\n");

      if (SensorStop == false) {
          status = sl_sleeptimer_start_periodic_timer_ms(
                                &SensorMeasure_timer,
                                SENSORMEASURE_TICK,
                                on_timer_callback,
                                (void *)NULL,
                                1,
                                SL_SLEEPTIMER_NO_HIGH_PRECISION_HF_CLOCKS_REQUIRED_FLAG);

          if(status != SL_STATUS_OK) printf("Timer not started.\r\n");
      }

      while (!SGP30warmup) {
          if (toggle_timeout == true) {
            toggle_timeout = false;
            sl_led_toggle(&sl_led_led0);
            measureAirQuality(&_CO2, &_TVOC);

            if (toggle_counter == 5 || toggle_counter == 10 || toggle_counter == 15) {
                get_dht11data(&_TestDHT11);
                printf("DHT11: %u\r\n", _TestDHT11);
            }

            if (toggle_counter >= 16) {
                SGP30warmup = true;
                toggle_counter = 0;
            }
          }
      }
      if (SensorStop == false) {
        status = sl_sleeptimer_stop_timer(&SensorMeasure_timer);
        if(status != SL_STATUS_OK) printf("Failed to stop timer.\r\n");
      }

      if (!measureAirQuality(&_CO2, &_TVOC)) {
          printf("We have some results for you...\r\n");
          allgood = true;
      }
      else printf("Sorry failed to get results :-(\r\n");
  }
  *CO2 = _CO2;
  *TVOC = _TVOC;
  SensorStop = true;
  return allgood;
}

/***************************************************************************//**
 * Sleeptimer timeout callback.
 ******************************************************************************/
static void on_timer_callback(sl_sleeptimer_timer_handle_t *handle,
                       void *data)
{
  (void)&handle;
  (void)&data;
  toggle_timeout = true;
  toggle_counter ++;
}

