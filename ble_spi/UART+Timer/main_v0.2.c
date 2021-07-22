/** @file
 *
 * @defgroup ble_sdk_uart_over_ble_main main.c
 * @{
 * @ingroup  ble_sdk_app_nus_eval
 * @brief    UART over BLE application main file.
 *
 * This file contains the source code for a sample application that uses the Nordic UART service.
 * This application uses the @ref srvlib_conn_params module.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "app_timer.h"
#include "ble_nus.h"
#include "app_uart.h"
#include "app_util_platform.h"
#include "bsp_btn_ble.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_delay.h"
#include "fds.h"
#include "nrf_fstorage.h"
#include "nrf_fstorage_sd.h"
#include "nrf_fstorage_nvmc.h"

#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "nrf_drv_spi.h"
#include "lis3dsh_drive.h"
#include "SEGGER_RTT.h"
#include "nrf_drv_clock.h"


#define APP_BLE_CONN_CFG_TAG            1                                           /**< A tag identifying the SoftDevice BLE configuration. */

#define DEVICE_NAME                     "Nordic_Ravi"                               /**< Name of device. Will be included in the advertising data. */
#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN                  /**< UUID type for the Nordic UART Service (vendor specific). */

#define APP_BLE_OBSERVER_PRIO           3                                           /**< Application's BLE observer priority. You shouldn't need to modify this value. */

#define APP_ADV_INTERVAL                64                                          /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */

#define APP_ADV_DURATION                12043                                       /**< The advertising duration (180 seconds) in units of 10 milliseconds. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(20, UNIT_1_25_MS)             /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(75, UNIT_1_25_MS)             /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(2043)                       /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                      /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define UART_TX_BUF_SIZE                256                                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                256                                         /**< UART RX buffer size. */

#define MS_TICKS                        2.5                                         /**< MS duration for app timer. */



/***  Used for testing FDS ***/
static volatile uint8_t write_flag_fds_test = 0; 
#define FILE_ID_FDS_TEST_1     0x1111
#define REC_KEY_FDS_TEST_1     0x2222

#define FILE_ID_FDS_TEST_2     0x4444
#define REC_KEY_FDS_TEST_2     0x5555

int32_t data_store[2043] = {0};
int32_t data_read[2043] = {0};
 //( 9*2043 -3)
int32_t MAX_SEND_LENGTH = 2043 ;// per page limit
int32_t DATA_STORE_LEN = 2043;
int j = 0;
int32_t intValueMgZ;
uint16_t pages = 1;  //page no
bool ready_to_write= false;
bool ready_to_read = false;
/*****************************/





BLE_NUS_DEF(m_nus, NRF_SDH_BLE_TOTAL_LINK_COUNT);                                   /**< BLE NUS service instance. */
NRF_BLE_GATT_DEF(m_gatt);                                                           /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                                             /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);                                                 /**< Advertising module instance. */
APP_TIMER_DEF(m_repeated_timer_id);                                                 /** <Handler for repeated timer event used to capture the data>**/
APP_TIMER_DEF(m_single_shot_timer_id);                                              /** <Handler for repeated timer event used to capture the data>**/


static uint16_t   m_conn_handle          = BLE_CONN_HANDLE_INVALID;                 /**< Handle of the current connection. */
static uint16_t   m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3;            /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */
static ble_uuid_t m_adv_uuids[]          =                                          /**< Universally unique service identifier. */
{
    {BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}
};


/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyse
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}



static void data_send(void)
{ 

          ret_code_t      err_code;
          int k = 0;

          printf("Ready to send data over BLE NUS");

         

          do
            {       static uint8_t data[4] ;
                    nrf_gpio_pin_toggle(LED_3); 
                    sprintf((char *)data ,"%d \r\n", data_store[k] );

                  
                           
                          //  NRF_LOG_HEXDUMP_DEBUG(data_array, sizeof(data);

                            do
                            {
                                uint16_t length = (uint16_t)sizeof(data);
                                err_code = ble_nus_data_send(&m_nus,data , &length, m_conn_handle);
                               // printf(data);
                                //nrf_delay_ms(300);

                                if ((err_code != NRF_ERROR_INVALID_STATE) &&
                                    (err_code != NRF_ERROR_RESOURCES) &&
                                    (err_code != NRF_ERROR_NOT_FOUND))
                                {
                                    APP_ERROR_CHECK(err_code);
                                }
                            } while (err_code == NRF_ERROR_RESOURCES);

                            k++;
            } while( k<2043);


            // preparing for sending data from 2nd page
            memset(data_store, 0, sizeof(data_store));
            pages++;

            if(pages==5){
               ready_to_read = false;
               pages = 1;
            }
            printf("data sent to BLE, :) %d/r/n", pages );
    
}
 

// ------------------------------------------------------------------------------------------------------
// ----------------------------FLASH---FDS--- START----------------------------------------------



static ret_code_t fds_test_write(void)
{
		
		//static uint32_t const m_deadbeef[2] = {0xDEADBEEF,0xBAADF00D};
		//static uint8_t const m_deadbeef[8] = {0xA,0xB,0xC,0xD};

                static uint8_t m_deadbeef[10] = {0};


		fds_record_t        record;
		fds_record_desc_t   record_desc;

		// Set up data.
               // static char const m_deadbeef[] = "1023 567 0979 890";
		
		
		// Set up record.
		record.file_id              = FILE_ID_FDS_TEST_1;
		record.key              = REC_KEY_FDS_TEST_1;
		record.data.p_data       = &m_deadbeef;
		//record.data.length_words   = sizeof(m_deadbeef)/sizeof(uint32_t);
		record.data.length_words   = sizeof(m_deadbeef)/sizeof(uint8_t);
				
		ret_code_t ret = fds_record_write(&record_desc, &record);
		if (ret != NRF_SUCCESS)        //changeing
		{
                // Handle error.
                  printf("fds_record_write failed. Line: %d, err_code: %d\r\n", __LINE__,ret);
				return ret;
		}
		 printf("Writing Record ID = %d \r\n",record_desc.record_id);
                

		return NRF_SUCCESS;
}



static ret_code_t fds_write(uint16_t pages)
{
		#define FILE_ID     0x1111
		#define REC_KEY     0x2222
		static int32_t m_deadbeef[2043] = {0} ; // changing size of deadbeef to match the size of total length of the array
                uint32_t write_counter = DATA_STORE_LEN ; 
               // uint32_t counter = 0;

              //  int32_t DATA_STORE_LEN = 18384 //( 9*2043 -3)
               // int32_t MAX_SEND_LENGTH = 2043 // per page limit


                // Copy the elements of data_store to the buffer variable
                memcpy(m_deadbeef, data_store, sizeof(m_deadbeef));

		fds_record_t        record;
		fds_record_desc_t   record_desc;

                // clear the data_store for reuse
                memset(data_store, 0, sizeof(data_store));

                printf("Write Time %d\r\n", pages);
             
                  record.file_id              = FILE_ID;
                  record.key                  = pages;
                  record.data.length_words = sizeof(m_deadbeef)/ sizeof(int32_t) ;
                 // record.data.p_data          = &m_deadbeef[DATA_STORE_LEN - write_counter]; //DATA_STORE_LEN = 2043
                  record.data.p_data          = &m_deadbeef;
                  ret_code_t ret = fds_record_write(&record_desc, &record);

                  if (ret != NRF_SUCCESS)
                  {    // Handle error.
                        printf("fds_record_write failed. Line: %d, err_code: %d\r\n", __LINE__,ret);
                        return ret;
                  } else {

                    printf("Writing Record ID = %d \r\n",record_desc.record_id);
                    //printf("Write Success :%d\t%d" , pages);
                    printf("\r\n");
                    //counter = counter + 2043;
                    //nrf_delay_ms(4000);
                    ret = NRF_SUCCESS;

                   }
                   

// ------------------------------------------Multipage write----------------------------------------------------------------
		
                //for ( uint16_t pages = 0x0001; pages < 0x0005; pages ++)

                //{

                //  printf("Write Loop %d\r\n", pages);
             
                //  record.file_id              = FILE_ID;
                //  record.key                  = pages;
               
                //  record.data.length_words = MAX_SEND_LENGTH;
                // // record.data.p_data          = &m_deadbeef[DATA_STORE_LEN - write_counter]; //DATA_STORE_LEN = 2043
                // record.data.p_data          = &m_deadbeef[counter];
                  
              
                //  ret_code_t ret = fds_record_write(&record_desc, &record);
                //  if (ret != NRF_SUCCESS)
                //  {
                //                  return ret;
                //  } else {
                //    printf("Write Success :%d\t%d" , pages, counter);
                //    printf("\r\n");
                //    counter = counter + 2043;
                //    nrf_delay_ms(4000);
                //   ret = NRF_SUCCESS;

                    //if(write_counter < MAX_SEND_LENGTH){
                    //  write_counter = 0;
                    //} else{
                    //  write_counter = write_counter - MAX_SEND_LENGTH;
                    //}
                    
                  
              //    }

               //   }

                              
// ------------------------------------------------------------------------------------------------------------      
         


		return NRF_SUCCESS;
}

static ret_code_t fds_test_read(void)
{

		fds_flash_record_t  flash_record;
		fds_record_desc_t   record_desc;
		fds_find_token_t    ftok ={0};//Important, make sure you zero init the ftok token
		//uint32_t *data;
		uint8_t *data;
                //char *data;
		uint32_t err_code;
		
		printf("Start searching... \r\n");
		// Loop until all records with the given key and file ID have been found.
		while (fds_record_find(FILE_ID_FDS_TEST_1, REC_KEY_FDS_TEST_1, &record_desc, &ftok) == NRF_SUCCESS)       //changeing
		{
				err_code = fds_record_open(&record_desc, &flash_record);

				if ( err_code != NRF_SUCCESS)       //changeing
				{
					return err_code;		
				}
				
				printf("Found Record ID = %d\r\n",record_desc.record_id);
				printf("Data = ");
				//data = (uint32_t *) flash_record.p_data;
				data = (uint8_t *) flash_record.p_data;
                                //data = (char *) flash_record.p_data;
				for (uint8_t i=0;i<flash_record.p_header->length_words;i++)
				{
					//printf("0x%8x",data[i]);
                                        printf("%d", data[i]);
                                        printf("\r\n");
				}
				
				// Access the record through the flash_record structure.
				// Close the record when done.
				err_code = fds_record_close(&record_desc);
				if (err_code != NRF_SUCCESS)        //changeing
				{
					return err_code;	
				}
		}
		return NRF_SUCCESS;
		
}


static ret_code_t fds_read(uint16_t pages)
{
		#define FILE_ID     0x1111
		#define REC_KEY     0x2222
		fds_flash_record_t  flash_record;
		fds_record_desc_t   record_desc;
		fds_find_token_t    ftok ={0};//Important, make sure you zero init the ftok token
		uint32_t *data;
		uint32_t err_code;
                int c = 0;
		
		printf("Start searching... \r\n");
		// Loop until all records with the given key and file ID have been found.


                printf("Read Loop : %d\r\n :", pages);
             
                //while (fds_record_find(FILE_ID, REC_KEY, &record_desc, &ftok) == NRF_SUCCESS)

		while (fds_record_find(FILE_ID, pages , &record_desc, &ftok) == NRF_SUCCESS)
		{
				err_code = fds_record_open(&record_desc, &flash_record);
				if ( err_code != NRF_SUCCESS)
				{
					return err_code;		
				}
				
				printf("Found Record ID = %d\r\n",record_desc.record_id);
				printf("Data = ");
				data = (uint32_t *) flash_record.p_data;
				for (uint32_t i=0;i<flash_record.p_header->length_words;i++)
				{       data_store[i] = data[i];
					//printf("0x%8x ",data[i]);
                                        printf("%d\t%d", i, data[i]);
                                        printf("\r\n");
				}

                               
				printf("\r\n");
				// Access the record through the flash_record structure.
				// Close the record when done.
				err_code = fds_record_close(&record_desc);

                                // copy the data to data_store and then call data_send
                               // memcpy(data_store, data, sizeof(data_store)); // for time being , store in data_store manually

                                data_send();
				if (err_code != NRF_SUCCESS)
				{
					return err_code;	
				}
		}

               // nrf_delay_ms(1000);
                //err_code = app_timer_start(m_single_shot_timer_id, APP_TIMER_TICKS(3000), NULL);
               // APP_ERROR_CHECK(err_code);


             

// ----------------------------Multipage read-------------------------------------------
  //           for( uint16_t pages = 1; pages < 5 ; pages++ ){
  //                printf("Read Loop : %d\r\n :", pages);
             
  //              //while (fds_record_find(FILE_ID, REC_KEY, &record_desc, &ftok) == NRF_SUCCESS)

		//while (fds_record_find(FILE_ID, pages , &record_desc, &ftok) == NRF_SUCCESS)
		//{
		//		err_code = fds_record_open(&record_desc, &flash_record);
		//		if ( err_code != NRF_SUCCESS)
		//		{
		//			return err_code;		
		//		}
				
		//		printf("Found Record ID = %d\r\n",record_desc.record_id);
		//		printf("Data = ");
		//		data = (uint32_t *) flash_record.p_data;
		//		for (uint32_t i=0;i<flash_record.p_header->length_words;i++)
		//		{       data_read[i+c] = data[i];
		//			//printf("0x%8x ",data[i]);
  //                                      printf("%d\t%d", i, data[i]);
  //                                      printf("\r\n");
		//		}
		//		printf("\r\n");
		//		// Access the record through the flash_record structure.
		//		// Close the record when done.
		//		err_code = fds_record_close(&record_desc);
		//		if (err_code != NRF_SUCCESS)
		//		{
		//			return err_code;	
		//		}
		//}

  //              c = c+2043;
  //              nrf_delay_ms(1000);
  //              //err_code = app_timer_start(m_single_shot_timer_id, APP_TIMER_TICKS(3000), NULL);
  //             // APP_ERROR_CHECK(err_code);


  //           }

 // ----------------------------Multipage read-------------------------------------------

		return NRF_SUCCESS;
		
}

static ret_code_t fds_test_find_and_delete (void)
{

		fds_record_desc_t   record_desc;
		fds_find_token_t    ftok;
	
		ftok.page=0;
		ftok.p_addr=NULL;
		// Loop and find records with same ID and rec key and mark them as deleted.
            
		while (fds_record_find(FILE_ID_FDS_TEST_1, REC_KEY_FDS_TEST_1, &record_desc, &ftok) == NRF_SUCCESS)        //changeing
             
		{
			fds_record_delete(&record_desc);
			printf("Deleted record ID: %d \r\n",record_desc.record_id);
		}
                
		// call the garbage collector to empty them, don't need to do this all the time, this is just for demonstration
		ret_code_t ret = fds_gc();

		if (ret != NRF_SUCCESS)       //changeing
		{
				return ret;
		}
		return NRF_SUCCESS;
}

static ret_code_t fds_find_and_delete (uint32_t file_id, uint32_t record_key)
{

		fds_record_desc_t   record_desc;
		fds_find_token_t    ftok;
	
		ftok.page=0;
		ftok.p_addr=NULL;
		// Loop and find records with same ID and rec key and mark them as deleted. 

            
		while (fds_record_find(file_id, record_key, &record_desc, &ftok) == NRF_SUCCESS)        //changeing
                //while (fds_record_find(file_id, pages, &record_desc, &ftok) == NRF_SUCCESS)        //changeing
		{
			fds_record_delete(&record_desc);
			printf("Deleted record ID: %d \r\n",record_desc.record_id);
		}
                
		// call the garbage collector to empty them, don't need to do this all the time, this is just for demonstration
		ret_code_t ret = fds_gc();

		if (ret != NRF_SUCCESS)       //changeing
		{
				return ret;
		}
		return NRF_SUCCESS;
}


// FDS Handler
static void my_fds_evt_handler(fds_evt_t const * p_fds_evt)
{

    ret_code_t err_code;
    switch (p_fds_evt->id)
    {
        case FDS_EVT_INIT:

        
            if (p_fds_evt->result != NRF_SUCCESS)//changeing
            {
                // Initialization failed.
            }
            break;
				case FDS_EVT_WRITE:
                         
						if (p_fds_evt->result == NRF_SUCCESS)       //changeing
						{
							write_flag_fds_test=1;
                                                       // printf( "Data written");
                                                      // printf("\r\n fds_read \r\n");
                                                      //err_code = fds_read();
                                                      //nrf_gpio_pin_toggle(LED_3);
						}
						break;
        default:
            break;
    }
}

static ret_code_t fds_test_init (void)
{
	
		ret_code_t ret = fds_register(my_fds_evt_handler);
		if (ret != NRF_SUCCESS)        //changeing
		{
                // Handle error
                        printf("Registering of the FDS event handler has failed. Line: %d, err_code: %d\r\n", __LINE__,ret);
                         APP_ERROR_CHECK(ret);
					return ret;
				
		}
		ret = fds_init();
		if (ret != NRF_SUCCESS)        //changeing
		{
                  // Handle error
                  printf("fds_init has failed. Line: %d, err_code: %d\r\n", __LINE__,ret);
                  APP_ERROR_CHECK(ret);
				return ret;
		}
		
		return NRF_SUCCESS;
		
}



// ------------------------------------------------------------------------------------------------------
// ----------------------------FLASH---FDS--- END----------------------------------------------


// ------------------------------------------------------------------------------------------------------
// ----------------------------ACCELEROMETER--DATA--- START----------------------------------------------




 static void acc_data(void)   //static void acc_data(int intValueMgZ , int i)
 {

   
   // int32_t read_data[2043] = {0};
    ret_code_t       err_code;
     
   
    printf(" \r\ninside acc_data\r\n ");
  //  nrf_delay_ms(2000);  // write function is getting delayed. 

  // Writing to Flash Process start
  

    printf("\r\n fds_find_and_delete \r\n");
    err_code = fds_find_and_delete(FILE_ID_FDS_TEST_1, pages);  // fds_find_and_delete(FILE_ID_FDS_TEST_1, REC_KEY_FDS_TEST_1);
    APP_ERROR_CHECK(err_code);
   



    
   for(int loop = 0; loop < 2044 ; loop++)
    {
      
      if( loop <2043 ) {
         data_store[loop] = twoComplToInt16(data_store[loop]);
         data_store[loop] = data_store[loop] * SENS_2G_RANGE_MG_PER_DIGIT ;
         }

      else{
         // sending for the first page

        printf("data_store modified /r/n");
        printf("\r\n fds_write \r\n");
        err_code =fds_write(pages);
        APP_ERROR_CHECK(err_code);

      //  while (write_flag_fds_test==0); // this will make it wait for the write operation to finish
        pages++;
        nrf_gpio_pin_toggle(LED_3);

      }

  
   
     }

      
    
    //// sending for the first page
    //err_code =fds_write();
    //APP_ERROR_CHECK(err_code);
    
    //printf("\r\n Wait for some time \r\n");
    //nrf_delay_ms(5000);
    
    //while (write_flag_fds_test==0);
    //printf("\r\n fds_read \r\n");
    //err_code = fds_read();
    //nrf_gpio_pin_toggle(LED_3);

   
    // if page = 5, stop the timer. wait for write operation to finish. enable read
     if (pages==5)
     {
      err_code = app_timer_stop(m_repeated_timer_id);
      APP_ERROR_CHECK(err_code);

      while (write_flag_fds_test==0);
      pages = 1;
      ready_to_write= false;
      ready_to_read = true;
      
      }

    
     
 }


 static void data_grab(){
       
            ret_code_t err_code;

         
                 // printf("\r\nNo: %d \r\n", j);  
                  intValueMgZ = ((LIS3DSH_read_reg(ADD_REG_OUT_Z_H) << 8) | LIS3DSH_read_reg(ADD_REG_OUT_Z_L));
                           
                  data_read[j] = intValueMgZ; // what if data_read fills up quicker than fds_write queue operation?
                  j++;  
                       
                
                

         

         if( j == 2043)  // stop the timer
         {     
              printf("data_read full/r/n");
              // Copy data_read values to data_store 
               memcpy(data_store, data_read, sizeof(data_store));

               // Reinitializing data_read to 0 for next value
               memset(data_read, 0, sizeof(data_read));
               j = 0; 
               
               // Enable one iteration of acc_data
               ready_to_write = true;
                

               
         }

       

        

 }

// ------------------------------------------------------------------------------------------------------
// ----------------------------ACCELEROMETER--DATA--- END----------------------------------------------


// ------------------------------------------------------------------------------------------------------
// ----------------------------TIMER--- START---------------------------------------------------------------------


/**@brief Function for initializing the timer module.
 */
static void timers_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}

static void repeated_timer_handler(void * p_context)
{
      ret_code_t err_code;
      // add the function which has to be performed repeatedly when the timer expires. 
      nrf_gpio_pin_toggle(LED_1);
      //printf(" here"); 
    //  intValueMgZ = ((LIS3DSH_read_reg(ADD_REG_OUT_Z_H) << 8) | LIS3DSH_read_reg(ADD_REG_OUT_Z_L));

      data_grab();
     
  
}




/**@brief Create timers.
 */
static void create_timers()
{
    ret_code_t err_code;

    // Create timers
    
    err_code = app_timer_create(&m_repeated_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                repeated_timer_handler);
    APP_ERROR_CHECK(err_code);

   
}

// ----------------------------TIMER---END---------------------------------------------------------------------
// --------------------------------------------------------------------------------------------------------
/**@brief Function for the GAP initialization.
 *
 * @details This function will set up all the necessary GAP (Generic Access Profile) parameters of
 *          the device. It also sets the permissions and appearance.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *) DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for handling the data from the Nordic UART Service.
 *
 * @details This function will process the data received from the Nordic UART BLE Service and send
 *          it to the UART module.
 *
 * @param[in] p_evt       Nordic UART Service event.
 */
/**@snippet [Handling the data received over BLE] */
static void nus_data_handler(ble_nus_evt_t * p_evt)
{

    if (p_evt->type == BLE_NUS_EVT_RX_DATA)
    {
        uint32_t err_code;

        NRF_LOG_DEBUG("Received data from BLE NUS. Writing data on UART.");
        NRF_LOG_HEXDUMP_DEBUG(p_evt->params.rx_data.p_data, p_evt->params.rx_data.length);

        for (uint32_t i = 0; i < p_evt->params.rx_data.length; i++)
        {
            do
            {
                err_code = app_uart_put(p_evt->params.rx_data.p_data[i]);
                if ((err_code != NRF_SUCCESS) && (err_code != NRF_ERROR_BUSY))
                {
                    NRF_LOG_ERROR("Failed receiving NUS message. Error 0x%x. ", err_code);
                    APP_ERROR_CHECK(err_code);
                }
            } while (err_code == NRF_ERROR_BUSY);
        }
        if (p_evt->params.rx_data.p_data[p_evt->params.rx_data.length - 1] == '\r')
        {
            while (app_uart_put('\n') == NRF_ERROR_BUSY);
        }
    }

}
/**@snippet [Handling the data received over BLE] */


/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    uint32_t           err_code;
    ble_nus_init_t     nus_init;  // Event handler for received data
    nrf_ble_qwr_init_t qwr_init = {0};

    // Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);

    // Initialize NUS.
    memset(&nus_init, 0, sizeof(nus_init));

    // Links up "nus_data_handler" function with this.
    nus_init.data_handler = nus_data_handler;

    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling an event from the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module
 *          which are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply setting
 *       the disconnect_on_fail config parameter, but instead we use the event handler
 *       mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling errors from the Connection Parameters module.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    uint32_t err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    uint32_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
            break;
        case BLE_ADV_EVT_IDLE:
            sleep_mode_enter();
            break;
        default:
            break;
    }
}


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t err_code;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            printf("Connected");
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            APP_ERROR_CHECK(err_code);
            
            // starting the timer
            err_code = app_timer_start(m_repeated_timer_id, APP_TIMER_TICKS(MS_TICKS), NULL);
            APP_ERROR_CHECK(err_code);

            break;

        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected");
            // LED indication will be changed when advertising starts.
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            // No implementation needed.
            break;
    }
}




/**@brief Function for the SoftDevice initialization.
 *
 * @details This function initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}


/**@brief Function for handling events from the GATT library. */
void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
    if ((m_conn_handle == p_evt->conn_handle) && (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED))
    {
        m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
        NRF_LOG_INFO("Data len is set to 0x%X(%d)", m_ble_nus_max_data_len, m_ble_nus_max_data_len);
    }
    NRF_LOG_DEBUG("ATT MTU exchange completed. central 0x%x peripheral 0x%x",
                  p_gatt->att_mtu_desired_central,
                  p_gatt->att_mtu_desired_periph);
}


/**@brief Function for initializing the GATT library. */
void gatt_init(void)
{
    ret_code_t err_code;

    err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
void bsp_event_handler(bsp_event_t event)
{
    uint32_t err_code;
    switch (event)
    {
        case BSP_EVENT_SLEEP:
            sleep_mode_enter();
            break;

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BSP_EVENT_WHITELIST_OFF:
            if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
            {
                err_code = ble_advertising_restart_without_whitelist(&m_advertising);
                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                }
            }
            break;

        default:
            break;
    }
}


/**@brief   Function for handling app_uart events.
 *
 * @details This function will receive a single character from the app_uart module and append it to
 *          a string. The string will be be sent over BLE when the last character received was a
 *          'new line' '\n' (hex 0x0A) or if the string has reached the maximum data length.
 */
/**@snippet [Handling the data received over UART] */




void uart_event_handle(app_uart_evt_t * p_event)
{ 
   
    static uint8_t data_array[30];
    static uint8_t index = 0;
    uint32_t       err_code;

    switch (p_event->evt_type)
    {
        case APP_UART_DATA_READY:
            UNUSED_VARIABLE(app_uart_get(&data_array[index]));
            index++;

            if ((data_array[index - 1] == '\n') ||
                (data_array[index - 1] == '\r') ||
                (index >= 20))
            {
                if (index > 1)
                {
                    NRF_LOG_DEBUG("Ready to send data over BLE NUS");
                    NRF_LOG_HEXDUMP_DEBUG(data_array, index);

                    do
                    {
                        uint16_t length = (uint16_t)index;
                        //err_code = sensor_data(&data_array);
                        //APP_ERROR_CHECK(err_code);

                        err_code = ble_nus_data_send(&m_nus, data_array, &length, m_conn_handle);
                        //err_code = ble_nus_data_send(&m_nus, "Rav", &length, m_conn_handle);
                        if ((err_code != NRF_ERROR_INVALID_STATE) &&
                            (err_code != NRF_ERROR_RESOURCES) &&
                            (err_code != NRF_ERROR_NOT_FOUND))
                        {
                            APP_ERROR_CHECK(err_code);
                        }
                    } while (err_code == NRF_ERROR_RESOURCES);
                }

                index = 0;
            }
            break;

        case APP_UART_COMMUNICATION_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_communication);
            break;

        case APP_UART_FIFO_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_code);
            break;

        default:
            break;
    }
} 
/**@snippet [Handling the data received over UART] */


/**@brief  Function for initializing the UART module.
 */
/**@snippet [UART Initialization] */
static void uart_init(void)
{
    uint32_t                     err_code;
    app_uart_comm_params_t const comm_params =
    {
        .rx_pin_no    = RX_PIN_NUMBER,
        .tx_pin_no    = TX_PIN_NUMBER,
        .rts_pin_no   = RTS_PIN_NUMBER,
        .cts_pin_no   = CTS_PIN_NUMBER,
        .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
        .use_parity   = false,
#if defined (UART_PRESENT)
        .baud_rate    = NRF_UART_BAUDRATE_115200
#else
        .baud_rate    = NRF_UARTE_BAUDRATE_115200
#endif
    };

    APP_UART_FIFO_INIT(&comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_event_handle,
                       APP_IRQ_PRIORITY_LOWEST,
                       err_code);
    APP_ERROR_CHECK(err_code);
}
/**@snippet [UART Initialization] */


/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    uint32_t               err_code;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));

    init.advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance = false;
    init.advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;

    init.srdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.srdata.uuids_complete.p_uuids  = m_adv_uuids;

    init.config.ble_adv_fast_enabled  = true;
    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout  = APP_ADV_DURATION;
    init.evt_handler = on_adv_evt;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}


/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(bool * p_erase_bonds)
{
    bsp_event_t startup_event;

    uint32_t err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}


/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}


/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
    uint32_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);
}




/**@brief Application main function.
 */


     

   

 
int main(void)
{
    bool erase_bonds;

    uint32_t       err_code;
    // Data capture variables
     int32_t intValueMgZ;
     int i = 0;
     
//-------------------------------------------------------------------------------------------------------------------
    // Initialize.

    // Initialize the pin numbers and baud rate etc for the connection
    uart_init();

    // Initializes the logger 
    log_init();

    // Timer module
    timers_init();
    buttons_leds_init(&erase_bonds);
    power_management_init();

    // Initializes the BLE softdevice
    ble_stack_init();

    // Initializes device name and connection times. Sets Generic Access Profile
    gap_params_init();

    // Initializes and sets MTU values
    gatt_init();

    // Links up UART and data and everything
    services_init();

    // Initializes advertising
    advertising_init();

    conn_params_init();

     //Call the SPI initialization function
    SPI_Init();

    // Start execution.
    printf("\r\nUART started.\r\n");
    printf("Debug logging for UART over RTT started.");
    advertising_start();
    LIS3DSH_init();

    printf("\r\n fds_test_inti \r\n");
    err_code =fds_test_init();
    APP_ERROR_CHECK(err_code);

    // Creating the timer to capture the data points
    create_timers();

// ------------------------------------------------------------------------------------

    nrf_gpio_pin_toggle(LED_2);

    // Enter main loop.
    nrf_gpio_pin_toggle(LED_3);
       
       // Data capture function
      // acc_data(intValueMgZ, i );
               
    for (;;)
    {
        idle_state_handle();
        
        if( ready_to_write){
        acc_data();
        ready_to_write= false;
        }

        if( ready_to_read){
          
        fds_read(pages);

        }

        
    }

}


/**
 * @}
 */
