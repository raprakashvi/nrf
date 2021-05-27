

#include <stdbool.h>
#include <stdint.h>
#include "nrf_delay.h"
#include "boards.h"


#include "nrf_drv_spi.h"
#include "lis3dsh_drive.h"

// Must include these headers to work with nrf logger
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"




int main(void)
{


    // Initialize the Logger module and check if any error occured during initialization
    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
	
    // Initialize the default backends for nrf logger
    NRF_LOG_DEFAULT_BACKENDS_INIT();

    // print the log msg over uart port
    NRF_LOG_INFO("This is log data from nordic device..");


    // variables to hold x y z values in mg
    int intValueMgX, intValueMgY, intValueMgZ;
	
    //Initialize the LEDs on board to use them
    bsp_board_init(BSP_INIT_LEDS); 
  
	
    //Call the SPI initialization function
    SPI_Init();
    //nrf_delay_ms(500);
    NRF_LOG_INFO("spi");
    printf(UC_WHO_AM_I_DEFAULT_VALUE);
    // Call the Lis3dsh initialization function
    LIS3DSH_init();
    NRF_LOG_INFO("lis");
   int i = 0;              
    while(true)
      {
		//
		intValueMgX = ((LIS3DSH_read_reg(ADD_REG_OUT_X_H) << 8) | LIS3DSH_read_reg(ADD_REG_OUT_X_L));
		intValueMgY = ((LIS3DSH_read_reg(ADD_REG_OUT_Y_H) << 8) | LIS3DSH_read_reg(ADD_REG_OUT_Y_L));
		intValueMgZ = ((LIS3DSH_read_reg(ADD_REG_OUT_Z_H) << 8) | LIS3DSH_read_reg(ADD_REG_OUT_Z_L));

		/* transform X value from two's complement to 16-bit int */
		intValueMgX = twoComplToInt16(intValueMgX);
		/* convert X absolute value to mg value */
		intValueMgX = intValueMgX * SENS_2G_RANGE_MG_PER_DIGIT ;

		/* transform Y value from two's complement to 16-bit int */
		intValueMgY = twoComplToInt16(intValueMgY);
		/* convert Y absolute value to mg value */
		intValueMgY = intValueMgY * SENS_2G_RANGE_MG_PER_DIGIT ;

		/* transform Z value from two's complement to 16-bit int */
		intValueMgZ = twoComplToInt16(intValueMgZ);
		/* convert Z absolute value to mg value */
		intValueMgZ = intValueMgZ * SENS_2G_RANGE_MG_PER_DIGIT ;
                //
		NRF_LOG_INFO("No=%6d X=%6d Y=%6d Z=%6d \r\n",i, intValueMgX, intValueMgY, intValueMgZ);
		nrf_delay_ms(300);
                i++;
		nrf_gpio_pin_toggle(LED_1);
      }
}   

