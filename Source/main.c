/* Includes ------------------------------------------------------------------*/
#include "assert.h"
#include "stddef.h"
#include "st_sigfox_api.h"
#include "sigfox_retriever.h"
#include "S2LP_SDK_Util.h"
#include "MCU_Interface.h"

//********************************
#include "main.h"   
#include <string.h> /* strlen */
//#include <stdio.h>  /* sprintf */
#include <math.h>   /* trunc */


#define USE_ENVI_SENSORS  //LPS22HB MEMS pressure sensor, absolute digital output barometer and HTS221 Capacitive digital relative humidity and temperature

#ifdef USE_ACC_GYRO
static void *LSM6DSL_X_0_handle = NULL;
static void *LSM6DSL_G_0_handle = NULL;
#endif
#ifdef USE_MAG_ACC
static void *LSM303AGR_X_0_handle = NULL;
static void *LSM303AGR_M_0_handle = NULL;
#endif
#ifdef USE_ENVI_SENSORS
static void *HTS221_H_0_handle  = NULL;
static void *HTS221_T_0_handle  = NULL;
static void *LPS22HB_P_0_handle  = NULL;
static void *LPS22HB_T_0_handle  = NULL;
#endif
/* Private function prototypes */

static void initializeAllSensors( void );
static void deinitializeAllSensors( void );
static void enableAllSensors( void );
static void disableAllSensors( void );
#ifdef USE_ENVI_SENSORS
static void floatToInt( float in, int32_t *out_int, int32_t *out_dec, int32_t dec_prec );
#endif

#ifdef USE_ACC_GYRO
static void Accelero_Sensor_Handler( void *handle );
static void Gyro_Sensor_Handler( void *handle );
#endif
#ifdef USE_MAG_ACC
static void Accelero_Sensor_Handler( void *handle );
static void Magneto_Sensor_Handler( void *handle );
#endif
#ifdef  USE_ENVI_SENSORS
static float Humidity_Sensor_Handler( void *handle );
static float Temperature_Sensor_Handler( void *handle );
static void Pressure_Sensor_Handler( void *handle );
#endif
//***************************

/* a flag to understand if the button has been pressed */
static volatile uint8_t but_pressed=0; 


void Fatal_Error(void)
{
  SdkEvalLedInit(LED1);
  
  while(1)
  {
    ST_LOWLEVEL_Delay(100);
    SdkEvalLedToggle(LED1);
  }
}


void Appli_Exti_CB(uint16_t GPIO_Pin)
{
  /* In this case the application EXTI event is the button pressed */
  if(GPIO_Pin==GPIO_PIN_13)
  {
    /* set the button pressed flag */
    but_pressed=1;
    
    /* Add more button press handling here if necessary by the user application */


    
    
    /******************************************/
    
  }
}

/**
* @brief  Configure all the GPIOs in low power mode.
* @param  None
* @retval None
*/
void enterGpioLowPower(void)
{
  /* Set all the GPIOs in low power mode (input analog) */
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStructure.Pin = GPIO_PIN_All;

  GPIO_InitStructure.Pin = GPIO_PIN_All & (~GPIO_PIN_13); //button
  HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);
  
  GPIO_InitStructure.Pin = GPIO_PIN_All & (~GPIO_PIN_13) & (~GPIO_PIN_14) 
                            & (~GPIO_PIN_5) & (~GPIO_PIN_2) & (~GPIO_PIN_3);//SWD, LED, UART
  HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_InitStructure.Pin = GPIO_PIN_All  & (~GPIO_PIN_8) & (~GPIO_PIN_9); //I2C1 pins
  HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  /* de init the SPI */
  SdkEvalSpiDeinit();
  
  /* keep the EXTI on the button pin */
  SdkEvalPushButtonInit(BUTTON_KEY,BUTTON_MODE_EXTI);
  
  /* keep the SDN driven */
  SdkEvalM2SGpioInit(M2S_GPIO_SDN,M2S_MODE_GPIO_OUT);
  
  /* Be sure that it is driving the device to be in shutdown*/
  SdkEvalEnterShutdown();
}

void exitGpioLowPower(void)
{ 
  /* Reinit the SDN pin and SPI */
  SdkEvalM2SGpioInit(M2S_GPIO_SDN,M2S_MODE_GPIO_OUT);
  SdkEvalSpiInit();
}


void main(void)
{  
  /* Instance some variables where the SigFox data of the board are stored */
  uint8_t pac[8];
  uint32_t id;
  uint8_t rcz, retr_err;
  
  /* Some variables to store the application data to transmit */
  uint32_t cust_counter=0;
	float customer_temp[12]={0};
  uint8_t customer_data[12]={0};
  uint8_t customer_resp[8];
  
  /* Initialize the hardware */
  HAL_Init();
  ST_LOWLEVEL_SetSysClock();
  SdkEvalIdentification();  
  SdkEvalM2SGpioInit(M2S_GPIO_SDN,M2S_MODE_GPIO_OUT);
  SdkEvalSpiInit();
  EepromSpiInitialization();
  
  //For X_NUCLEO_IKS01A2 ********
  USARTConfig();  /* Initialize UART */
  initializeAllSensors(); /* Initialize sensors */
  enableAllSensors();
  //****************************  
  
  /* Identify the RF board reading some production data */
  S2LPManagementIdentificationRFBoard();
  
  /* Put the radio off */
  SdkEvalEnterShutdown();
      
  /* Give the RF offset to the library */
  ST_MANUF_API_set_freq_offset(S2LPManagementGetOffset());
  
  /* The low level driver uses the internal RTC as a timer while the STM32 is in low power.
     This function calibrates the RTC using an auxiliary general purpose timer in order to 
     increase its precision. */
  ST_LOWLEVEL_TimerCalibration(500);
  
  /* Initialize the blue push button on the board */
  SdkEvalPushButtonInit(BUTTON_KEY,BUTTON_MODE_EXTI);

  /* Retrieve the SigFox data from the board - ID, PAC and RCZ used */
  retr_err=enc_utils_retrieve_data(&id,pac,&rcz);
  
  /* If the retriever returned an error (code different from RETR_OK) the application must not continue */
  if(retr_err!=RETR_OK){
   /* Print error message */
    PRINTF("Board not ready for SigFox. Missing board ID/PAC.\n\r");
    PRINTF("Please register the board with ST to obtain the ID/PAC.\n\r");
    Fatal_Error();
  }
  
  /* Here it is necessary to understand if we are using the code for ETSI or FCC */
#ifdef FOR_ETSI
  /* In case of ETSI we should stuck if RCZ is not 1 */
  if(rcz!=1)
    Fatal_Error();
  
  /* RCZ1 - open the SigFox library */
  if(ST_SIGFOX_API_open(ST_RCZ1,(uint8_t*)&id)!=0)
  {
    /* Stuck in case of error */
    Fatal_Error();
  }
#elif FOR_FCC
  
  uint8_t send_call_num=0;
  
  if(rcz==2)
  {
    /* RCZ2 - open the SigFox library */
    if(ST_SIGFOX_API_open(ST_RCZ2,(uint8_t*)&id)!=0)
    {
      /* Stuck in case of error */
      Fatal_Error();
    }
     
    /* In FCC we can choose the macro channel to use by a 86 bits bitmask
       In this case we use the first 9 macro channels */
    uint32_t config_words[3]={0x1FF,0,0};
    
    /* Set the standard configuration with default channel to 1 */
    if(ST_SIGFOX_API_set_std_config(config_words,1)!=0)
    {
      /* Stuck in case of error */
      Fatal_Error();
    }
    ST_SIGFOX_API_reset();
  }
  else if(rcz==4)
  {
    /* RCZ4 - open the SigFox library */
    if(ST_SIGFOX_API_open(ST_RCZ4,(uint8_t*)&id)!=0)
    {
      /* Stuck in case of error */
      Fatal_Error();
    }
    
    /* In FCC we can choose the macro channel to use by a 86 bits bitmask
       In this case we use 9 consecutive macro channels starting from 63 (920.8MHz) */
    uint32_t config_words[3]={0,0xF0000000,0x1F};
    
    /* Set the standard configuration with default channel to 63 */
    if(ST_SIGFOX_API_set_std_config(config_words,63)!=0)
    {
      /* Stuck in case of error */
      Fatal_Error();
    }
    ST_SIGFOX_API_reset();

  }
  else
  {
    /* Stuck the application if we are using the FW for FCC and RCZ is not 2 or 4 */
    Fatal_Error();
  }
#endif

  /* application main loop */
    /* go in low power with the STM32 waiting for an external interrupt */
    // PRINTF("\r\nTest.\n\r");
    
    disableAllSensors();  
    HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON,PWR_STOPENTRY_WFI);
    ST_LOWLEVEL_SetSysClock();
    exitGpioLowPower();
    enableAllSensors();
    
		while(1)
  {
		SdkDelayMs(10000);
		PRINTF("-----------------------\r\n");
	
  
      //For X_NUCLEO_IKS01A2 ********
      /* Perform handlers */
#ifdef USE_ACC_GYRO
      Accelero_Sensor_Handler( LSM6DSL_X_0_handle );
      Gyro_Sensor_Handler( LSM6DSL_G_0_handle );
#endif
#ifdef USE_MAG_ACC
      Accelero_Sensor_Handler( LSM303AGR_X_0_handle );
      Magneto_Sensor_Handler( LSM303AGR_M_0_handle );
#endif
#ifdef USE_ENVI_SENSORS
      customer_temp[0] = Humidity_Sensor_Handler( HTS221_H_0_handle );
			PRINTF("Humidity = %f\r\n",customer_temp[0]);
      customer_data[0] = roundf(customer_temp[0]);

      customer_temp[1] = Temperature_Sensor_Handler( HTS221_T_0_handle );
			PRINTF("Temp = %f\r\n",customer_temp[1]);
      customer_data[1] = roundf(customer_temp[1]) + 128;
      
      ST_SIGFOX_API_send_frame(customer_data, 2,customer_resp, 0, 0);

#endif
  
#ifdef FOR_FCC
      /* Only for RCZ2 and 4. Since the SigFox base stations are able to receive only on the default ch
      (1 for RCZ2 and 63 for RCZ4), we send an API reset in order to be sure that all the packets will be 
      received by the sigfox network. In this way, the duty cycle will be not ensured by library.
      The user should perfom the TX operation every a minimum of 20s. */
      if(send_call_num==1)
      {
        ST_SIGFOX_API_reset();
        send_call_num=0;
      }
      else
      {
        send_call_num++;
      }
#endif
  
      

    
  }
  
}
#ifdef USE_ENVI_SENSORS
/**
 * @brief  Splits a float into two integer values.
 * @param  in the float value as input
 * @param  out_int the pointer to the integer part as output
 * @param  out_dec the pointer to the decimal part as output
 * @param  dec_prec the decimal precision to be used
 * @retval None
 */
static void floatToInt( float in, int32_t *out_int, int32_t *out_dec, int32_t dec_prec )
{

  *out_int = (int32_t)in;
  if(in >= 0.0f)
  {
    in = in - (float)(*out_int);
  }
  else
  {
    in = (float)(*out_int) - in;
  }
  *out_dec = (int32_t)trunc(in * pow(10, dec_prec));
}
#endif

#ifdef USE_ENVI_SENSORS
/**
 * @brief  Handles the humidity data getting/sending
 * @param  handle the device handle
 * @retval None
 */
static float Humidity_Sensor_Handler( void *handle )
{

  int32_t d1, d2;
  uint8_t id;
  float humidity;
  uint8_t status;

  BSP_HUMIDITY_Get_Instance( handle, &id );

  BSP_HUMIDITY_IsInitialized( handle, &status );

  if ( status == 1 )
  {
    if ( BSP_HUMIDITY_Get_Hum( handle, &humidity ) == COMPONENT_ERROR )
    {
      humidity = 0.0f;
    }
    
    /* Perform post processing of sensor data */
    
    
#if 1
    /* Print sensor data for debuging purposes */
    floatToInt( humidity, &d1, &d2, 2 );
//    sprintf( dataOut, "\r\nHUM[%d]: %d.%02d\r\n", (int)id, (int)d1, (int)d2 );
//    HAL_UART_Transmit( &UartHandle, ( uint8_t * )dataOut, strlen( dataOut ), 5000 );
    
    PRINTF("Humidity: %d.%01d \r\n", (int)d1, (int)d2 );
    
    return humidity;
    
#endif
    
  }
}



/**
 * @brief  Handles the temperature data getting/sending
 * @param  handle the device handle
 * @retval None
 */
static float Temperature_Sensor_Handler( void *handle )
{

  int32_t d1, d2;
  uint8_t id;
  float temperature;
  uint8_t status;

  BSP_TEMPERATURE_Get_Instance( handle, &id );

  BSP_TEMPERATURE_IsInitialized( handle, &status );

  if ( status == 1 )
  {
    if ( BSP_TEMPERATURE_Get_Temp( handle, &temperature ) == COMPONENT_ERROR )
    {
      temperature = 0.0f;
    }
    
    /* Perform post processing of sensor data */
    
    
    
    
    
    /* ************************************** */
    
    
    
#if 1
    /* Print sensor data for debuging purposes */
    floatToInt( temperature, &d1, &d2, 2 );
//    sprintf( dataOut, "\r\nTEMP[%d]: %d.%02d\r\n", (int)id, (int)d1, (int)d2 );
//    HAL_UART_Transmit( &UartHandle, ( uint8_t * )dataOut, strlen( dataOut ), 5000 );

    PRINTF("Temperature: %d.%1d\r\n", (int)d1, (int)d2 );
    
    return temperature;
#endif
    
  }
}

#endif


/**
 * @brief  Initialize all sensors
 * @param  None
 * @retval None
 */
static void initializeAllSensors( void )
{
  deinitializeAllSensors();
  
#ifdef USE_ACC_GYRO 
  BSP_ACCELERO_Init( LSM6DSL_X_0, &LSM6DSL_X_0_handle );
  BSP_GYRO_Init( LSM6DSL_G_0, &LSM6DSL_G_0_handle );
#endif
#ifdef USE_MAG_ACC
  BSP_ACCELERO_Init( LSM303AGR_X_0, &LSM303AGR_X_0_handle );
  BSP_MAGNETO_Init( LSM303AGR_M_0, &LSM303AGR_M_0_handle );
#endif
#ifdef USE_ENVI_SENSORS
  BSP_HUMIDITY_Init( HTS221_H_0, &HTS221_H_0_handle );
  BSP_TEMPERATURE_Init( HTS221_T_0, &HTS221_T_0_handle );
//  BSP_TEMPERATURE_Init( LPS22HB_T_0, &LPS22HB_T_0_handle ); //optional
  BSP_PRESSURE_Init( LPS22HB_P_0, &LPS22HB_P_0_handle );
#endif
}

/**
 * @brief  Initialize all sensors
 * @param  None
 * @retval None
 */
static void deinitializeAllSensors( void )
{

#ifdef USE_ACC_GYRO 
  BSP_ACCELERO_DeInit(&LSM6DSL_X_0_handle );
  BSP_GYRO_DeInit(&LSM6DSL_G_0_handle );
#endif
#ifdef USE_MAG_ACC
  BSP_ACCELERO_DeInit(&LSM303AGR_X_0_handle );
  BSP_MAGNETO_DeInit(&LSM303AGR_M_0_handle );
#endif
#ifdef USE_ENVI_SENSORS
  BSP_HUMIDITY_DeInit(&HTS221_H_0_handle );
  BSP_TEMPERATURE_DeInit(&HTS221_T_0_handle );
//  BSP_TEMPERATURE_DeInit(&LPS22HB_T_0_handle ); //optional
  BSP_PRESSURE_DeInit(&LPS22HB_P_0_handle );
#endif
}

/**
 * @brief  Enable all sensors
 * @param  None
 * @retval None
 */
static void enableAllSensors( void )
{

#ifdef USE_ACC_GYRO 
  BSP_ACCELERO_Sensor_Enable( LSM6DSL_X_0_handle );
  BSP_GYRO_Sensor_Enable( LSM6DSL_G_0_handle );
#endif
#ifdef USE_MAG_ACC
  BSP_ACCELERO_Sensor_Enable( LSM303AGR_X_0_handle );
  BSP_MAGNETO_Sensor_Enable( LSM303AGR_M_0_handle );
#endif
#ifdef USE_ENVI_SENSORS
  BSP_HUMIDITY_Sensor_Enable( HTS221_H_0_handle );
  BSP_TEMPERATURE_Sensor_Enable( HTS221_T_0_handle );
//  BSP_TEMPERATURE_Sensor_Enable( LPS22HB_T_0_handle );
  BSP_PRESSURE_Sensor_Enable( LPS22HB_P_0_handle );
#endif
  
  SdkDelayMs(10);
}



/**
 * @brief  Disable all sensors
 * @param  None
 * @retval None
 */
static void disableAllSensors( void )
{

#ifdef USE_ACC_GYRO   
  BSP_ACCELERO_Sensor_Disable( LSM6DSL_X_0_handle );
  BSP_GYRO_Sensor_Disable( LSM6DSL_G_0_handle );
#endif
#ifdef USE_MAG_ACC
  BSP_ACCELERO_Sensor_Disable( LSM303AGR_X_0_handle );
  BSP_MAGNETO_Sensor_Disable( LSM303AGR_M_0_handle );
#endif
#ifdef USE_ENVI_SENSORS
  BSP_HUMIDITY_Sensor_Disable( HTS221_H_0_handle );
  BSP_TEMPERATURE_Sensor_Disable( HTS221_T_0_handle );
//  BSP_TEMPERATURE_Sensor_Disable( LPS22HB_T_0_handle ); //optional
  BSP_PRESSURE_Sensor_Disable( LPS22HB_P_0_handle );
#endif
}



#ifdef  USE_FULL_ASSERT
/**
* @brief  Reports the name of the source file and the source line number
*         where the assert_param error has occurred.
* @param file: pointer to the source file name
* @param line: assert_param error line source number
* @retval : None
*/
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number */
  printf("Wrong parameters value: file %s on line %d\r\n", file, line);
  
  /* Infinite loop */
  while (1)
  {
  }
}
#endif
