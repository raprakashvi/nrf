
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "app_error.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "lis3dsh_drive.h"
#include "nrf_drv_spi.h"


// Must include these headers to work with nrf logger
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"




#define SPI_BUFSIZE  8   //SPI Communication buffer size
#define SPI_INSTANCE  0 //SPI Instance to be used

uint8_t   spi_tx_buf[SPI_BUFSIZE]; // spi tx buffer 
uint8_t   spi_rx_buf[SPI_BUFSIZE]; // spi rx buffer



volatile  uint8_t   SPIReadLength, SPIWriteLength; // variables to hold read and write lengths

static volatile bool spi_xfer_done;  /* Flag used to indicate that SPI instance completed the transfer. */
static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);  /* SPI instance. */

/* A function to convert two's complement value to 16-bit int value */
int twoComplToInt16(int twoComplValue)
{
	  int int16Value = 0;

	  /* conversion */
	  if(twoComplValue > 32768)
	  {
		    int16Value = -(((~twoComplValue) & 0xFFFF) + 1);
	  }
	  else
	  {
		    int16Value = twoComplValue;
	  }

	  return int16Value;
}




/* A function to write into LIS3DSH Internal Register */
static void LIS3DSH_write_reg(int reg, int data)
{
    SPIWriteLength = 2; // set the spi write length to 2 bytes
    SPIReadLength = 0; // set the read length
    
    spi_tx_buf[0] = SET_WRITE_SINGLE_CMD(reg); // set the first byte which is a write command
    spi_tx_buf[1] = data; // A byte of data to be sent
	
    /* set the transfer flag to false to indicate data transfer has not yet started */
    spi_xfer_done = false;
    
    /* transfer the data by calling the spi transfer function and call it in app error check so if
       any error occurs it will catch it and sends it on debug port */
    APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, spi_tx_buf, SPIWriteLength, spi_rx_buf, SPIReadLength));

    /* wait until the transfer is completed */
    while(!spi_xfer_done); // this flag will be set to true in the spi interrupt handler function
}




/* A function to read a value from the internal register of lis3dsh */
int LIS3DSH_read_reg(int reg)
{	
    /* Set the read command for reading a single byte */
    spi_tx_buf[0] = SET_READ_SINGLE_CMD(reg);
    
    /* Set the transfer flag to false */
    spi_xfer_done = false;
    
    /* call the spi transfer function */
    APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, spi_tx_buf, 2, spi_rx_buf, 2));
    
    while(spi_xfer_done == false){};

    return spi_rx_buf[1];
}



/* SPI Event handler function, on every transfer this event call back function
will be executed upon the intterupts from spi */
void spi_event_handler(nrf_drv_spi_evt_t const * p_event,
                       void *                    p_context)
{ /* Set the data transfer flag to true to indicate the data transmission has finished */ 
  spi_xfer_done = true;
}



/* A function to initialize SPI Instance */
void SPI_Init(void)
{
    /* Create a struct to hold the configurations and set these values to default */
    nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;

    /* Assign pins to SPI instance */
    spi_config.ss_pin   = 28;
    spi_config.miso_pin = 29;
    spi_config.mosi_pin = 30;
    spi_config.sck_pin  = 31;

    /* Configure the transfer speed by setting the clock for data transmission */
    spi_config.frequency  = NRF_DRV_SPI_FREQ_4M;

    /* Call the SPI initialization function with in APP_ERROR_CHECK function so that if any error
     occurs during initialization then we can get the response on debug window */
    APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, spi_event_handler, NULL));
}



/* A function to initialize the Lis3dsh Sensor */
void LIS3DSH_init(void)
{
    
    int intRegValue;

    /* Read the who am i register to confirm lis3dsh is communicating with our nrf device */
    intRegValue = LIS3DSH_read_reg(ADD_REG_WHO_AM_I);
   
    /* check the who am i register value and compare it with default who am i value for lis3dsh 
    if who am i is correct then proced */
    if( intRegValue == UC_WHO_AM_I_DEFAULT_VALUE )
      {
	// set output data rate to 400 Hz and enable X,Y,Z axis */
	LIS3DSH_write_reg(ADD_REG_CTRL_1, UC_ADD_REG_CTRL_1_CFG_VALUE);
        //LIS3DSH_write_reg(ADD_REG_CTRL_1, 0x57);
        LIS3DSH_write_reg(ADD_REG_CTRL_2, UC_ADD_REG_CTRL_2_CFG_VALUE);
        LIS3DSH_write_reg(ADD_REG_CTRL_3, UC_ADD_REG_CTRL_3_CFG_VALUE);
        LIS3DSH_write_reg(ADD_REG_CTRL_4, UC_ADD_REG_CTRL_4_CFG_VALUE);
        LIS3DSH_write_reg(ADD_REG_CTRL_5, UC_ADD_REG_CTRL_5_CFG_VALUE);
        LIS3DSH_write_reg(ADD_REG_CTRL_6, UC_ADD_REG_CTRL_6_CFG_VALUE);


        

	
        /* verify written value */
	intRegValue = LIS3DSH_read_reg(ADD_REG_CTRL_1);
	
        /* if written value is different */
	 if( intRegValue != UC_ADD_REG_CTRL_1_CFG_VALUE )
          {
            /* ERROR: stay here... */
	    while(1)
             {
		printf("Write Reg ERR\r\n");
		nrf_delay_ms(500);
              }
          } 
      }

    else
    {
      /* ERROR: stay here... */
      while(1)
	{
           printf("Device is not exist\r\n");
           printf(UC_WHO_AM_I_DEFAULT_VALUE);
           nrf_delay_ms(500);
	}
			   
    }
}









