/* USER CODE BEGIN Header */

/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

// Firmware for the LED Driver Board -  Mach III revision
// Changes history
//      Oct.   14, 2025 - Rolf S - ver. 0.2.0: add some safety checks
//      Sept   16, 2025 - Rolf S - ver. 0.1.1: cleanup and imported to github
//      August 10, 2025 - Rolf S - ver. 0.1.0: ported to M3,
// currently, left 3 digits need to be in range of 0..9 only see (*1*)
#define VERSION_MAYOR 0x0
#define VERSION_MINOR 0x2
#define VERSION_PATCH 0x0

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>
#include <string.h>
#define LIGHTMODBUS_SLAVE
#define LIGHTMODBUS_SLAVE_FULL
#define LIGHTMODBUS_IMPL
#include <lightmodbus/lightmodbus.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef enum {false, true} bool;
typedef enum {chA, chB, both} DAC_ch;
typedef enum {board, internal, external} trig_source;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define N_BUF 32 // size of the serial buffers
#define N_COMM 32 // size of the buffer to store last command

#define LED_FEB_MB_COIL_BASE     1   // coil               1 bit  / rw
#define LED_FEB_MB_DISC_BASE 10001   // discrete input     1 bits / ro
#define LED_FEB_MB_INPR_BASE 30001   // input registers   16 bits / ro
#define LED_FEB_MB_HOLD_BASE 40001   // holding registers 16 bits / rw

// coils
#define C_POWER_ON            0
#define C_TRIGGER_ENABLE      1
#define C_LED_BIAS_ENABLE     2
#define C_FIRST_PULSE_FIX     3
#define C_RESET_ERROR_COUNT   4
#define POWER_ON              LED_FEB_MB_COIL_BASE+C_POWER_ON
#define TRIGGER_ENABLE        LED_FEB_MB_COIL_BASE+C_TRIGGER_ENABLE
#define LED_BIAS_ENABLE       LED_FEB_MB_COIL_BASE+C_LED_BIAS_ENABLE
#define FIRST_PULSE_FIX       LED_FEB_MB_COIL_BASE+C_FIRST_PULSE_FIX
#define RESET_ERROR_COUNT     LED_FEB_MB_COIL_BASE+C_RESET_ERROR_COUNT

// discrete input
#define D_POWER_STATUS        0
#define POWER_STATUS          LED_FEB_MB_DESC_BASE+D_POWER_STATUS

// input registers (mostly errors)
#define R_FIRMWARE_VERSION      0
#define R2_LED_BIAS_ADC_READ     1
#define R2_IMON_ADC_READ         2
#define R_LED_CURRENT_ERROR     3
#define R_LED_BIAS_ERROR        4
#define R_TRIG_SOURCE_ERR       5
#define R_LED_FEB_MB_SLAVE_ERR  6
#define R_LED_FEB_MB_GLOBAL_ERR 7
#define FIRMWARE_VERSION        LED_FEB_MB_INPR_BASE+R_FIRMWARE_VERSION
#define LED_BIAS_ADC_READ       LED_FEB_MB_INPR_BASE+R2_LED_BIAS_ADC_READ
#define IMON_ADC_READ           LED_FEB_MB_INPR_BASE+R2_IMON_ADC_READ
#define LED_CURRENT_ERROR       LED_FEB_MB_INPR_BASE+R_LED_CURRENT_ERROR
#define LED_BIAS_ERROR          LED_FEB_MB_INPR_BASE+R_LED_BIAS_ERROR
#define TRIG_SOURCE_ERR         LED_FEB_MB_INPR_BASE+R_TRIG_SOURCE_ERR
#define LED_FEB_MB_SLAVE_ERR    LED_FEB_MB_INPR_BASE+R_LED_FEB_MB_SLAVE_ERR
#define LED_FEB_MB_GLOBAL_ERR   LED_FEB_MB_INPR_BASE+R_LED_FEB_MB_GLOBAL_ERR

// holding registers
#define H_TRIGGER_SOURCE        0
#define H_LED_CHANNELS          1
#define H_LED_BIAS_DAC_SET      2
#define H_ALARM_THRESHOLD       3
#define H_uC_FLASH_FREQ         4
#define H_LIGHTMODBUS_SLAVE_ID  5
#define TRIGGER_SOURCE          LED_FEB_MB_HOLD_BASE+H_TRIGGER_SOURCE
#define LED_CHANNELS            LED_FEB_MB_HOLD_BASE+H_LED_CHANNELS
#define LED_BIAS_DAC_SET        LED_FEB_MB_HOLD_BASE+H_LED_BIAS_DAC_SET
#define ALARM_THRESHOLD         LED_FEB_MB_HOLD_BASE+H_ALARM_THRESHOLD
#define uC_FLASH_FREQ           LED_FEB_MB_HOLD_BASE+H_uC_FLASH_FREQ
#define LIGHTMODBUS_SLAVE_ID    LED_FEB_MB_HOLD_BASE+H_LIGHTMODBUS_SLAVE_ID

#define USE_SERIAL_1 1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

DAC_HandleTypeDef hdac;

UART_HandleTypeDef hlpuart1;
UART_HandleTypeDef huart1;

TIM_HandleTypeDef htim22;

osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */

uint16_t bias_adc[5];
uint16_t imon_adc[5];

uint32_t value_adc;	// ADC1, 2 testing

bool timer_on = false; // flag for the timer used to generate the pulses
trig_source trig_src = board; // variable holds the trigger source
uint32_t trig_rate_fact; // variable used to scale the PWM frequency

uint8_t buf_1[N_BUF]; // data buffer for uart1
uint32_t write_1_pos = 0; // write pointer for uart1

// RS modbus - just used until ID is set externally !
#define SLAVE_ADDRESS 20

// registers are 16 bits wide !
#define REG_COUNT 16
static uint16_t registers[REG_COUNT];
static uint16_t inputRegisters[REG_COUNT];
static uint8_t coils[REG_COUNT / 8];          // Each coil corresponds to one bit
static uint8_t discreteInputs[REG_COUNT / 8]; // Each input corresponds to one bit

// Next, the slave structure has to be initialized:
ModbusSlave slave;

// RS enough I hope ...
#define RXFRAMESIZE 16
uint8_t rxframe[RXFRAMESIZE];
uint8_t length;

uint16_t LED_Pins[] = {  LED_OE1_Pin, LED_OE2_Pin, LED_OE3_Pin, LED_OE4_Pin, LED_OE5_Pin, LED_OE6_Pin, LED_OE7_Pin };
void* LED_GPIO[]    = {  LED_OE1_GPIO_Port,  LED_OE2_GPIO_Port,  LED_OE3_GPIO_Port,  LED_OE4_GPIO_Port,
			 LED_OE5_GPIO_Port,  LED_OE6_GPIO_Port,  LED_OE7_GPIO_Port };

// indicator to trigger power on/off in startDefaultTask
uint8_t power_changed=false;

// read / write slave ID from / to non-volotile RAM, based on
// https://community.st.com/t5/stm32-mcus-products/how-to-used-internal-eeprom-read-and-write-operation-of/td-p/460192
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define DATA_EEPROM_BASE_ADDR ((uint32_t)0x08080000) /* Data EEPROM base address */
#define DATA_EEPROM_END_ADDR ((uint32_t)0x080807FF) /* Data EEPROM end address */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
static void MX_DAC_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM22_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */

char msg[] = "000 ";
#if USE_SERIAL_1
// prints a C string on hlpuart1
void serial_1_print(char * message)
{
  if (message != NULL) {
    HAL_UART_Transmit(&hlpuart1, (uint8_t *)(message), strlen(message), 300);
  }
  return;
}

// prints a C string on hlpuart1 followed by return line control character
void serial_1_println(char * message)
{
  if (message != NULL) {
    HAL_UART_Transmit(&hlpuart1, (uint8_t *)(message), strlen(message), 300);
    HAL_UART_Transmit(&hlpuart1, (uint8_t *)("\r\n"), 2, 100);
  }
  return;
}
#endif

ModbusErrorInfo err;

// default callback function for received character interrupts
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size_buffer)
{
  // RS Use Ser #if USE_SERIAL_1
  if (huart == &hlpuart1) {
    write_1_pos++;
    HAL_UARTEx_ReceiveToIdle_IT(&hlpuart1, (buf_1 + write_1_pos%N_BUF), 1);
  }
  else {
  // RS Use Ser #endif
    if (huart == &huart1) {
      err = modbusParseRequestRTU(&slave,
				  registers[LIGHTMODBUS_SLAVE_ID - LED_FEB_MB_HOLD_BASE],
				  rxframe, size_buffer);
      
      // Handle 'serious' errors such as memory allocation problems
      if (modbusGetGeneralError(err))
        registers[R_LED_FEB_MB_GLOBAL_ERR]=err.error;
      // Handle errors related to the structure of the request frame
      // i.e. frames meant for other slaves, invalid CRC etc.
      // Usually, though, you want to simply ignore these.
      if (modbusGetRequestError(err))
        {
	  registers[R_LED_FEB_MB_SLAVE_ERR]=err.error;
	  // handleRequestErrors(err);
#if USE_SERIAL_1
	  serial_1_println("modbus error");
#endif
	}
      
      // If the function did not return an error, the response can be accessed and is ready to be sent
      // to the master. The response frame can be acquired using modbusSlaveGetResponse()
      // and is modbusSlaveGetResponseLength() bytes long. Beware that response frame can be empty
      // in some cases!
      if (modbusIsOk(err))
        HAL_UART_Transmit(&huart1, modbusSlaveGetResponse(&slave), modbusSlaveGetResponseLength(&slave), 1000);
      
      HAL_UARTEx_ReceiveToIdle_IT(&huart1, rxframe, RXFRAMESIZE);
    }
    // RS Use Ser #if USE_SERIAL_1
    }
  // RS Use Ser #endif
  return;
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* Unlock the EEPROM: */
void UnlockEeprom(void)
{
  int loop=0;
  /* Wait for FLASH to be free */
  while (((FLASH->SR & FLASH_SR_BSY) != 0) && loop < 5000)
    {
      /*   insert timeout test */
      __WFI();
      loop++;
#if USE_SERIAL_1
      serial_1_println("Unlocking EEPROM !!!");
#endif
    }
  /* If PELOCK is locked */ 
  if ((FLASH->PECR & FLASH_PECR_PELOCK) != 0)
    {
      /* Unlock PELOCK */
      FLASH->PEKEYR = FLASH_PEKEY1;
      FLASH->PEKEYR = FLASH_PEKEY2;
    }
  /* enable flash interrupts */
  FLASH->PECR = FLASH->PECR | (FLASH_PECR_ERRIE | FLASH_PECR_EOPIE);
}

/**
 * Brief This function programs a word of data EEPROM.
 * The ERASE bit and DATA bit are cleared in PECR at the beginning
 * words are automatically erased if required before programming
 * Param addr is the 32-bit EEPROM address to program, data is the 32 bit word to program
 * Retval None
 */
void EepromProgram_16(uint32_t addr, uint16_t ee_data)
{
  /* NOTE: The EEPROM must be unlocked and the flash interrupts must have
     been enabled prior to calling this function.*/
  *(uint16_t *)(addr) = ee_data;
  /* write data to EEPROM */
  // __WFI();
#if USE_SERIAL_1
  serial_1_println("AFT 1 ERROR WRITIN TO EEPROM !!!");
#endif
  if (*(uint16_t *)(addr) != ee_data)
    {
      // error |= ERROR_PROG_32B_WORD;
#if USE_SERIAL_1
      serial_1_println("ERROR WRITIN TO EEPROM !!!");
#endif
    }
}

/* Lock the EEPROM: */
void LockEeprom(void)
{
  int loop=0;
  /* Wait for FLASH to be free */
  while (((FLASH->SR & FLASH_SR_BSY) != 0) && loop < 500)
    {
      /*   insert timeout test */
      __WFI();
      loop++;
    }
  /* disable flash interrupts */
  FLASH->PECR = FLASH->PECR & ~(FLASH_PECR_ERRIE | FLASH_PECR_EOPIE);
  /* Lock memory with PELOCK */
  FLASH->PECR = FLASH->PECR | FLASH_PECR_PELOCK;
}

uint16_t Read_SID_from_EEPROM()
{
  uint16_t d = *(__IO uint16_t *)DATA_EEPROM_BASE_ADDR;
  
  /* #if USE_SERIAL_1
     for ( uint16_t l=0; l<d; ++l)
     serial_1_println("READING FROM FLASH !!!");
     HAL_Delay(250);
     #endif
  */
  // no error detection
  return d;
}
void Transfer_SID_to_EEPROM(uint16_t sid)
{
  /* Unlock the EEPROM and enable flash interrupts */
  UnlockEeprom();
#if USE_SERIAL_1
  serial_1_println("unlocked 1 !!!");
#endif
  
  // UnlockEeprom();
  /* Reset the ERASE and DATA
     bits in the FLASH_PECR register to disable any residual erase */
  FLASH->PECR = FLASH->PECR & ~(FLASH_PECR_ERASE | FLASH_PECR_DATA);
  
#if USE_SERIAL_1
  serial_1_println("unlocked 2 !!!");
#endif
  /* Put the next line in a loop if sequential bits to be written with i as loop counter */
  EepromProgram_16(DATA_EEPROM_BASE_ADDR, sid);
  
#if USE_SERIAL_1
  serial_1_println("unlocked 3 !!!");
#endif
  LockEeprom(); /* Lock the EEPROM */
  
#if USE_SERIAL_1
  serial_1_println("WROTE TO FLASH !!!");
#endif
}

void FixFirstPulse()
{
  // fix first pulse issue
  // OE for trigger
  HAL_GPIO_WritePin (TRIG_OE_GPIO_Port,   TRIG_OE_Pin,   1);
  // select uC as trigger source
  HAL_GPIO_WritePin (TRIG_SEL1_GPIO_Port, TRIG_SEL1_Pin, 0);
  HAL_GPIO_WritePin (TRIG_SEL0_GPIO_Port, TRIG_SEL0_Pin, 1);
  
  // toggle trigger output pin few times
  HAL_GPIO_WritePin (uC_OUT_TRIG_GPIO_Port, uC_OUT_TRIG_Pin, 1);
  HAL_GPIO_WritePin (uC_OUT_TRIG_GPIO_Port, uC_OUT_TRIG_Pin, 0);
  HAL_GPIO_WritePin (uC_OUT_TRIG_GPIO_Port, uC_OUT_TRIG_Pin, 1);
  HAL_GPIO_WritePin (uC_OUT_TRIG_GPIO_Port, uC_OUT_TRIG_Pin, 0);

  // reset trigger input
  HAL_GPIO_WritePin (TRIG_SEL1_GPIO_Port, TRIG_SEL1_Pin, 0);
  HAL_GPIO_WritePin (TRIG_SEL0_GPIO_Port, TRIG_SEL0_Pin, 0);

  // will interfere currently with later code.
  // HAL_GPIO_WritePin (TRIG_OE_GPIO_Port,  TRIG_OE_Pin,        0);
}

void DAC_Value_SetLow()
{
  uint32_t dac_value = 0xFFF; // Value to load into DAC
  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, dac_value);  // write to DAC holding register
  HAL_DAC_Start(&hdac, DAC_CHANNEL_1);  // software trigger
  osDelay(10);
}

void PowerOn()
{
  // disable all LEDs
  for (uint32_t bit = 0; bit < 7; bit++)
    {
      HAL_GPIO_WritePin (LED_GPIO[bit], LED_Pins[bit], 0 );
    }
  // OLD: write_DAC_values(chA, 0xB76, 0x000); // set LED bias to about 6 V
  // now:
  DAC_Value_SetLow();
  /*  uint32_t dac_value = 0x0FFF; // minimum Value to load into DAC
      HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, dac_value);  // write to DAC holding register
      HAL_DAC_Start(&hdac, DAC_CHANNEL_1);  // software trigger */
  // Power Enable Sequence
#if USE_SERIAL_1
  serial_1_println("in PON 0");
#endif
  HAL_GPIO_WritePin (SW_3V3_ENABLE_GPIO_Port, SW_3V3_ENABLE_Pin, 1);
#if USE_SERIAL_1
  serial_1_println("in PON 1");
#endif
  osDelay(100);
#if USE_SERIAL_1
  serial_1_println("in PON 2");
#endif
  HAL_GPIO_WritePin (SW_5V_ENABLE_GPIO_Port, SW_5V_ENABLE_Pin, 1);
#if USE_SERIAL_1
  serial_1_println("in PON 3");
#endif
  osDelay(100);
#if USE_SERIAL_1
  serial_1_println("in PON 4");
#endif
  HAL_GPIO_WritePin (BOOST_ENABLE_GPIO_Port, BOOST_ENABLE_Pin, 1);
#if USE_SERIAL_1
  serial_1_println("in PON 5");
#endif
  osDelay(100);
#if USE_SERIAL_1
  serial_1_println("in PON 6");
#endif
  osDelay(100);
#if USE_SERIAL_1
  serial_1_println("in PON 7");
#endif
  // FixFirstPulse();
  if (modbusMaskRead(coils, C_FIRST_PULSE_FIX))
    {
      FixFirstPulse();
    }
#if USE_SERIAL_1
  serial_1_println("in PON 8");
#endif
  /* moved to trigger source selection
  // Setup timer for pulsing
  __HAL_TIM_SET_AUTORELOAD(&htim22, (32 / 2) - 1); // provide some default rate (1MHz)
  __HAL_TIM_SET_COMPARE(&htim22, TIM_CHANNEL_2, (32 / 4) - 1);
  HAL_TIM_PWM_Start(&htim22, TIM_CHANNEL_2); * /
#if USE_SERIAL_1
  serial_1_println("in PON 9");
#endif
  */
#if USE_SERIAL_1
  serial_1_println("Board power is ON");
#endif
  
  modbusMaskWrite(discreteInputs, 0, 1);
}

void PowerOff()
{
  // disable all LEDs
  for (uint32_t bit = 0; bit < 7; bit++)
    {
      HAL_GPIO_WritePin (LED_GPIO[bit], LED_Pins[bit], 0 );
    }
  DAC_Value_SetLow();
  /* uint32_t dac_value = 0xFFF; // Value to load into DAC
     HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, dac_value);  // write to DAC holding register
     HAL_DAC_Start(&hdac, DAC_CHANNEL_1);  // software trigger
     osDelay(100); */
  HAL_GPIO_WritePin (BOOST_ENABLE_GPIO_Port, BOOST_ENABLE_Pin, 0);
  osDelay(100);
  HAL_GPIO_WritePin (SW_5V_ENABLE_GPIO_Port, SW_5V_ENABLE_Pin, 0);
  osDelay(100);
  HAL_GPIO_WritePin (SW_3V3_ENABLE_GPIO_Port, SW_3V3_ENABLE_Pin, 0);
#if USE_SERIAL_1
  serial_1_println("Board power is OFF");
#endif
  modbusMaskWrite(discreteInputs, 0, 0);
}

void ADC_Read_values(uint16_t *bias, uint16_t *imon)
{
  HAL_ADC_Start(&hadc);
  uint8_t ret = HAL_ADC_PollForConversion(&hadc, 100 /*timeout*/);
  *bias = HAL_ADC_GetValue(&hadc);
  HAL_ADC_Start(&hadc);
  ret += HAL_ADC_PollForConversion(&hadc, 100 /*timeout*/);
  *imon = HAL_ADC_GetValue(&hadc);
  HAL_ADC_Stop(&hadc);
  inputRegisters[R2_LED_BIAS_ADC_READ] = *bias;
  inputRegisters[R2_IMON_ADC_READ] = *imon;
  if(ret !=0)
    inputRegisters[R_LED_BIAS_ERROR]++;
}

ModbusError myRegisterCallback(
    const ModbusSlave *status,
    const ModbusRegisterCallbackArgs *args,
    ModbusRegisterCallbackResult *result)
{
  uint16_t new_index=args->index;
  switch(args->type)
    {
    case MODBUS_HOLDING_REGISTER:
      new_index=args->index - LED_FEB_MB_HOLD_BASE;
      // there's one exception, and that's setting the SLAVE_ID,
      //  which goes into a holding register,
      //  but is at index "0". All other holding registers start
      //  at index 40001. We also accept
      //  this command only if the old SLAVE_ID is 20, the default
      if ( ( args->index == 0 ) && ( registers[H_LIGHTMODBUS_SLAVE_ID] == 20 ) )
	{
	  registers[H_LIGHTMODBUS_SLAVE_ID] = args->value;
	  Transfer_SID_to_EEPROM(registers[H_LIGHTMODBUS_SLAVE_ID]);
	}
      break;
    case MODBUS_INPUT_REGISTER:
      new_index=args->index - LED_FEB_MB_INPR_BASE;
      break;
    case MODBUS_COIL:
      new_index=args->index - LED_FEB_MB_COIL_BASE;
      break;
    case MODBUS_DISCRETE_INPUT:
      new_index=args->index - LED_FEB_MB_DISC_BASE;
      break;
    }
  switch (args->query)
    {
      // R/W access check
    case MODBUS_REGQ_R_CHECK:
    case MODBUS_REGQ_W_CHECK:
      // If result->exceptionCode of a read/write access query is not MODBUS_EXCEP_NONE,
      // an exception is reported by the slave. If result->exceptionCode is not set,
      // the behavior is undefined.
      inputRegisters[9]=args->type;
      inputRegisters[10]=new_index;
      result->exceptionCode = new_index < REG_COUNT ? MODBUS_EXCEP_NONE : MODBUS_EXCEP_ILLEGAL_ADDRESS;
      break;
      
      // Read register
      // no need to execute own code, just reading registers
    case MODBUS_REGQ_R:
      switch (args->type)
	{
        case MODBUS_HOLDING_REGISTER:
	  // deal with all cases - compiler would complain otherwise ...
	  switch(args->index)
	    {
            case TRIGGER_SOURCE:
            case LED_CHANNELS:
            case LED_BIAS_DAC_SET:
	    case LIGHTMODBUS_SLAVE_ID:
	      break;
	    }
          result->value = registers[new_index];
          break;
        case MODBUS_INPUT_REGISTER:
#if USE_SERIAL_1
	  // serial_1_println("in callback r ir");
#endif
	  switch(args->index)
	    {
            case LED_BIAS_ADC_READ:
	    case IMON_ADC_READ:
	      /* uint8_t ret=ADC_Read();
		 if(ret !=0)
		 inputRegisters[R_LED_BIAS_ERROR]++; */
	      // nop as ADC values now read out frequently,
	      //  with error counter increased if necessary
	      break;
	    }
          result->value = inputRegisters[new_index];
          break;
        case MODBUS_COIL:
#if USE_SERIAL_1
	  // serial_1_println("in callback r c");
#endif
          result->value = modbusMaskRead(coils, new_index);
          break;
        case MODBUS_DISCRETE_INPUT:
#if USE_SERIAL_1
	  // serial_1_println("in callback r dc");
#endif
          result->value = modbusMaskRead(discreteInputs, new_index);
          break;
	}
      break;
      
      // Write register, react to changes
    case MODBUS_REGQ_W:
#if USE_SERIAL_1
      //   serial_1_println("in callback w");
#endif
      switch (args->type)
	{
        case MODBUS_HOLDING_REGISTER:
#if USE_SERIAL_1
	  // serial_1_println("in callback w hr");
#endif
          registers[args->index - LED_FEB_MB_HOLD_BASE] = args->value;
          switch(args->index)
	    {
            case TRIGGER_SOURCE:
              // trigger source can only be 0, 1 or 2
              switch (args->value)
		{
                case 1:
		  // uC as source, set some timer as default (to remove it from PowerOn()
		  // Setup timer for pulsing
		  // provide some default rate (1MHz)
		  //		  __HAL_TIM_SET_AUTORELOAD(&htim22, (32 / 2) - 1);
		  //		  __HAL_TIM_SET_COMPARE(&htim22, TIM_CHANNEL_2, (32 / 4) - 1);
		  if ( registers[H_uC_FLASH_FREQ] >= 1 )
		    {
		      __HAL_TIM_SET_AUTORELOAD(&htim22, registers[H_uC_FLASH_FREQ] - 1);
		      __HAL_TIM_SET_COMPARE(&htim22, TIM_CHANNEL_2, registers[H_uC_FLASH_FREQ] / 2 - 1);
		      HAL_TIM_PWM_Start(&htim22, TIM_CHANNEL_2);
		    }
		  else
		    {
		      HAL_TIM_PWM_Stop(&htim22, TIM_CHANNEL_2);
		      // wrong input, increase error count
		      inputRegisters[R_TRIG_SOURCE_ERR]++;
		    }
		  /*  else
		      {
		      // default 1MHz
		      __HAL_TIM_SET_AUTORELOAD(&htim22, (32 / 2) - 1);
		      __HAL_TIM_SET_COMPARE(&htim22, TIM_CHANNEL_2, (32 / 4) - 1);
		      HAL_TIM_PWM_Start(&htim22, TIM_CHANNEL_2);
		      } */
                case 0:
                case 2:
		  HAL_GPIO_WritePin (TRIG_SEL1_GPIO_Port, TRIG_SEL1_Pin, (args->value & 2) > 0);
		  HAL_GPIO_WritePin (TRIG_SEL0_GPIO_Port, TRIG_SEL0_Pin, (args->value & 1) > 0);
		  HAL_TIM_PWM_Stop(&htim22, TIM_CHANNEL_2);
		  break;
                default:
		  // wrong input, increase error count
                  inputRegisters[R_TRIG_SOURCE_ERR]++;
                  break;
		}
              break;
            case LED_CHANNELS:
              uint8_t bit;
              for ( bit=0; bit<7; ++bit)
                HAL_GPIO_WritePin (LED_GPIO[bit], LED_Pins[bit], ( args->value & ( 1 << bit) ) > 0 );
	      for ( int s=0; s<5; ++s )
		imon_adc[s]=0;
              break;
            case LED_BIAS_DAC_SET:
	      // Configure and write to internal DAC
	      uint32_t dac_value = 0x0;
	      // update dac_value to load into DAC
	      dac_value=registers[H_LED_BIAS_DAC_SET];
	      HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, dac_value);  // write to DAC holding register
	      HAL_DAC_Start(&hdac, DAC_CHANNEL_1);  // software trigger
	      break;
	    case LIGHTMODBUS_SLAVE_ID:
	      // done already
	      // registers[H_LIGHTMODBUS_SLAVE_ID] = args->value;
	      Transfer_SID_to_EEPROM(registers[H_LIGHTMODBUS_SLAVE_ID]);
	    }
          break;
        case MODBUS_COIL:
#if USE_SERIAL_1
	  // serial_1_println("in callback w c");
#endif
          modbusMaskWrite(coils, args->index - LED_FEB_MB_COIL_BASE, args->value);
          switch(args->index)
	    {
            case POWER_ON:
#if USE_SERIAL_1
	      // serial_1_println("in callback w c P0");
#endif
	      // TTT
	      power_changed=true;
	      // FIXME: rest below can go, logic in StartDefaulTask
	      if (args->value)
		{
#if USE_SERIAL_1
		  serial_1_println("in callback w c P1");
#endif
		  // power on will be issued later, in StartDefaultTask as it can
		  //  take time for the full task to finish
		  // TTT PowerOn();
#if USE_SERIAL_1
		  serial_1_println("in callback w c P1b");
#endif
		}
              else
		{
#if USE_SERIAL_1
		  serial_1_println("in callback w c P2");
#endif
		  // power off will be issued later, in StartDefaultTask as it can
		  //  take time for the full task to finish
		  // TTT PowerOff();
#if USE_SERIAL_1
		  serial_1_println("in callback w c P2b");
#endif
		}
#if USE_SERIAL_1
	      serial_1_println("in callback w c P3");
#endif
              break;
            case TRIGGER_ENABLE:
#if USE_SERIAL_1
	      serial_1_println("in callback w c T");
#endif
	      HAL_GPIO_WritePin (TRIG_OE_GPIO_Port,
                                 TRIG_OE_Pin, args->value);
              break;
            case LED_BIAS_ENABLE:
#if USE_SERIAL_1
	      serial_1_println("in callback w c L");
#endif
	      HAL_GPIO_WritePin (BIAS_ENABLE_LED_GPIO_Port,
				 BIAS_ENABLE_LED_Pin, args->value);
              break;
            case FIRST_PULSE_FIX:
#if USE_SERIAL_1
	      serial_1_println("in callback w c FFP");
#endif
	      // nothing to do, coil already set
              break;
            case RESET_ERROR_COUNT:
#if USE_SERIAL_1
	      serial_1_println("in callback w c REC");
#endif
	      for( int reg=R_LED_CURRENT_ERROR; reg <=R_LED_FEB_MB_GLOBAL_ERR; ++reg )
		inputRegisters[reg]=0;
              break;
	    }
          break;
        default:
          break;
	}
      break;
    }
#if USE_SERIAL_1
  //  serial_1_println("in callback done");
#endif
  
  // Always return MODBUS_OK
  return MODBUS_OK;
}
/*
void handleSlaveError(LIGHTMODBUS_RET_ERROR err)
{
   registers[14]=err.error;
}

void handleRequestErrors(LIGHTMODBUS_RET_ERROR err)
{
   registers[15]=err.error;
}
*/
// Next, the slave structure has to be initialized:
ModbusSlave slave;

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  
  inputRegisters[11]=LED_FEB_MB_COIL_BASE;
  inputRegisters[12]=LED_FEB_MB_DISC_BASE;
  inputRegisters[13]=LED_FEB_MB_INPR_BASE;
  inputRegisters[14]=LED_FEB_MB_HOLD_BASE;
  inputRegisters[R_FIRMWARE_VERSION] =
    0x100*VERSION_MAYOR+
    0x010*VERSION_MINOR+
    0x001*VERSION_PATCH;
  
  /* USER CODE END 1 */
  
  /* MCU Configuration--------------------------------------------------------*/
  
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  
  /* USER CODE BEGIN Init */
  
  /* USER CODE END Init */
  
  /* Configure the system clock */
  SystemClock_Config();
  
  /* USER CODE BEGIN SysInit */
  
  /* USER CODE END SysInit */
  
  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC_Init();
  MX_DAC_Init();
  MX_LPUART1_UART_Init();
  MX_USART1_UART_Init();
  MX_TIM22_Init();
  /* USER CODE BEGIN 2 */
  
  // default hardware initialization .... mostly disable everything
  HAL_GPIO_WritePin (uC_OUT_TRIG_GPIO_Port, uC_OUT_TRIG_Pin, 0); // disables external trigger
  HAL_GPIO_WritePin (TRIG_OE_GPIO_Port, TRIG_OE_Pin, 0); // disables output
  HAL_GPIO_WritePin (TRIG_SEL1_GPIO_Port, TRIG_SEL1_Pin, 0); // part of selecting input b00, main board
  HAL_GPIO_WritePin (TRIG_SEL0_GPIO_Port, TRIG_SEL0_Pin, 0); // part of selecting input b00, main board
  trig_src = board; // flags the trigger source set
  HAL_GPIO_WritePin (LED_OE1_GPIO_Port, LED_OE1_Pin, 0); // disables trigger of LED1 / channel 0
  HAL_GPIO_WritePin (LED_OE2_GPIO_Port, LED_OE2_Pin, 0); // disables trigger of LED2 / channel 1
  HAL_GPIO_WritePin (LED_OE3_GPIO_Port, LED_OE3_Pin, 0); // disables trigger of LED3 / channel 2
  HAL_GPIO_WritePin (LED_OE4_GPIO_Port, LED_OE4_Pin, 0); // disables trigger of LED4 / channel 3
  HAL_GPIO_WritePin (LED_OE5_GPIO_Port, LED_OE5_Pin, 0); // disables trigger of LED5 / channel 4
  HAL_GPIO_WritePin (LED_OE6_GPIO_Port, LED_OE6_Pin, 0); // disables trigger of LED6 / channel 5
  HAL_GPIO_WritePin (LED_OE7_GPIO_Port, LED_OE7_Pin, 0); // disables trigger of LED7 / channel 6
  
  // IMON reset disabled
  HAL_GPIO_WritePin (IMON_RESET_GPIO_Port, IMON_RESET_Pin, 0);
  
  // Make sure LED bias regulator is disabled
  HAL_GPIO_WritePin (BIAS_ENABLE_LED_GPIO_Port, BIAS_ENABLE_LED_Pin, 0);
    
  // read out SLAVE_ID from EEPROM and save it, or set it to default if zero
  registers[H_LIGHTMODBUS_SLAVE_ID] = Read_SID_from_EEPROM();
  if ( registers[H_LIGHTMODBUS_SLAVE_ID] == 0)
    registers[H_LIGHTMODBUS_SLAVE_ID] = SLAVE_ADDRESS;
  
  // IMON reset configured for 'latch' mode
  HAL_Delay(100);  // power stabilize
  /*   HAL_GPIO_WritePin (IMON_ALERT_RESET_GPIO_Port, IMON_ALERT_RESET_Pin, 1); // Set to latch current over threshold   
   */
#if USE_SERIAL_1
  // start the reception of characters on both uarts
  HAL_UART_Receive_IT(&hlpuart1, (buf_1 + write_1_pos%N_BUF), 1);
  
  // print messages about the software and firmware for the user
  serial_1_print("Mach III LED driver board version ");
  /* currently, left 3 digits need to be in range of 0..9 only (*1*) */
  
  char ver_id[]="0.0.0";
  ver_id[0]+=VERSION_MAYOR;
  ver_id[2]+=VERSION_MINOR;
  ver_id[4]+=VERSION_PATCH;
  
  serial_1_println(ver_id);
  
  serial_1_print(" Slave_ID ");
  if ( registers[LIGHTMODBUS_SLAVE_ID - LED_FEB_MB_HOLD_BASE] > 100)
      serial_1_println("XX");
  else
    {
      char csid[]="00";
      csid[0]+=(registers[LIGHTMODBUS_SLAVE_ID - LED_FEB_MB_HOLD_BASE] / 10);
      csid[1]+=registers[LIGHTMODBUS_SLAVE_ID - LED_FEB_MB_HOLD_BASE] % 10;
      serial_1_println(csid);
    }
#endif
  // set the pulse duration to a large number
  trig_rate_fact = (uint32_t)32000 & 0xFFFF;
  // provide some default rate, 32 Mhz / 32,000 = 1 kHz
  __HAL_TIM_SET_AUTORELOAD(&htim22, (trig_rate_fact / 2) - 1);
  __HAL_TIM_SET_COMPARE(&htim22, TIM_CHANNEL_2, (trig_rate_fact / 4) - 1);
  timer_on = false;
  
  // Init slave
  err = modbusSlaveInit(
	   &slave,
	   myRegisterCallback,              // Callback for register operations
	   NULL, // myExceptionCallback,    // Callback for handling slave exceptions (optional)
	   modbusDefaultAllocator,          // Memory allocator for allocating responses
	   modbusSlaveDefaultFunctions,     // Set of supported functions
	   modbusSlaveDefaultFunctionCount  // Number of supported functions
			);
  
  // Check for errors
  // assert(modbusIsOk(err) && "modbusSlaveInit() failed");
  if(!modbusIsOk(err))
    {
      registers[R_LED_FEB_MB_GLOBAL_ERR]=err.error;
#if USE_SERIAL_1
      serial_1_println("Modbus error");
#endif
    }
#if USE_SERIAL_1
  serial_1_println("modbus ok");
#endif
  
  // start once waiting for interupts, will be then re-newed in callback
  HAL_UARTEx_ReceiveToIdle_IT(&huart1, rxframe, RXFRAMESIZE);
#if USE_SERIAL_1
  serial_1_println("uart1 ok");
#endif
  
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    osDelay(1000);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_4;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_LPUART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Lpuart1ClockSelection = RCC_LPUART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.OversamplingMode = DISABLE;
  hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = ENABLE;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerFrequencyMode = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = ENABLE;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */

  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief TIM22 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM22_Init(void)
{

  /* USER CODE BEGIN TIM22_Init 0 */

  /* USER CODE END TIM22_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM22_Init 1 */

  /* USER CODE END TIM22_Init 1 */
  htim22.Instance = TIM22;
  htim22.Init.Prescaler = 0;
  htim22.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim22.Init.Period = 65535;
  htim22.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim22.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim22) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim22, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim22) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim22, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim22, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM22_Init 2 */

  /* USER CODE END TIM22_Init 2 */
  HAL_TIM_MspPostInit(&htim22);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, TRIG_OE_Pin|TRIG_SEL0_Pin|TRIG_SEL1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, BOOST_ENABLE_Pin|SW_5V_ENABLE_Pin|
		    BIAS_ENABLE_LED_Pin|IMON_RESET_Pin|
		    LED_OE5_Pin, GPIO_PIN_RESET);
  
  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_OE1_Pin|LED_OE2_Pin|LED_OE3_Pin|LED_OE6_Pin
                          |LED_OE7_Pin|LED_OE4_Pin|SW_3V3_ENABLE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : TRIG_OE_Pin TRIG_SEL0_Pin TRIG_SEL1_Pin */
  GPIO_InitStruct.Pin = TRIG_OE_Pin|TRIG_SEL0_Pin|TRIG_SEL1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : BOOST_ENABLE_Pin SW_5V_ENABLE_Pin BIAS_ENABLE_LED_Pin IMON_RESET_Pin
                           LED_OE5_Pin */
  GPIO_InitStruct.Pin = BOOST_ENABLE_Pin|SW_5V_ENABLE_Pin|BIAS_ENABLE_LED_Pin
    |IMON_RESET_Pin|LED_OE5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_OE1_Pin LED_OE2_Pin LED_OE3_Pin LED_OE6_Pin
                           LED_OE7_Pin LED_OE4_Pin SW_3V3_ENABLE_Pin */
  GPIO_InitStruct.Pin = LED_OE1_Pin|LED_OE2_Pin|LED_OE3_Pin|LED_OE6_Pin
                          |LED_OE7_Pin|LED_OE4_Pin|SW_3V3_ENABLE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : CS_ADC_Pin SCLK_ADC_Pin SDATA_ADC_Pin */
  GPIO_InitStruct.Pin = CS_ADC_Pin|SCLK_ADC_Pin|SDATA_ADC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : IMON_N_ALERT_Pin */
  GPIO_InitStruct.Pin = IMON_N_ALERT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(IMON_N_ALERT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : uC_IN_TRIG_Pin */
  GPIO_InitStruct.Pin = uC_IN_TRIG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(uC_IN_TRIG_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// test for alarm in bias current too high.
// condition:
//  no LED enabled, but current
//  or flashes triggered.
void test_for_alarm()
{
  if ( ( 5 * registers[H_ALARM_THRESHOLD] <
       imon_adc[0]+
       imon_adc[1]+
       imon_adc[2]+
       imon_adc[3]+
       imon_adc[4] ) &&
       registers[H_LED_CHANNELS] == 0 )
    inputRegisters[R_LED_CURRENT_ERROR]++;
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  char c='0';
  /* Infinite loop */
  for(;;)
  {
    osDelay(100);
    c++;
    if ( c > '9' )
      c='0';
    msg[0]=c;
    if(power_changed)
      {
	if (modbusMaskRead(coils, C_POWER_ON) )
	  {
	    msg[2]='1';
	    PowerOn();
	  }
	else
	  {
	    msg[2]='0';
	    PowerOff();
	  }
	power_changed=false;
      }
#if USE_SERIAL_1
    serial_1_print(msg);
    serial_1_println(" in StartDefaultTask");
    // reduce time here
    // add loop over ADC read imon
    for ( int i=0; i<40; ++i)
      {
	/* for ( int l(0); l<4; ++l)
	   {
	   bias_adc[l+1]=bias_adc[l];
	   imon_adc[l+1]=imon_adc[l];
	   }
	ADC_Read_values(bias_adc[0], imon_adc[0]); */
	ADC_Read_values(&bias_adc[i % 5], &imon_adc[i % 5]);
	test_for_alarm();
	osDelay(25);
      }
    // osDelay(1000);
    // 
#endif
  }
  /* USER CODE END 5 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
