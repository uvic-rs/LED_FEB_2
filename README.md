# LED_FEB_2
firmware for the LED_FEB in the Hyper-K experiment.

##Coils
| function | Modbus addresses |
| ------------- | ------------- |
| POWER_ON         |          1 |
| TRIGGER_ENABLE   |          2 |
| LED_BIAS_ENABLE  |          3 |


## Discrete Inputs
| function | Modbus addresses |
| ------------- | ------------- |
| POWER_STATUS |        10001 |

## Input Registers
| function | Modbus addresses |
| ------------- | ------------- |
| R_FIRMWARE_VERSION      | 30001 |
| R_LED_CURRENT_ERROR     | 30002 |
| R_LED_BIAS_ERROR        | 30003 |
| R_TRIG_SOURCE_ERR       | 30004 |
| R_LED_FEB_MB_SLAVE_ERR  | 30005 |
| R_LED_FEB_MB_GLOBAL_ERR | 30006 |

## Holding Registers
| function | Modbus addresses |
| ------------- | ------------- |
| O_TRIGGER_SOURCE        | 40001 |
| O_LED_CHANNELS          | 40002 |
| O_LED_BIAS_DAC_SET      | 40003 |
| O_LED_BIAS_ADC_READ     | 40004 |
| O_IMON_ADC_READ         | 40005 |
| O_LIGHTMODBUS_SLAVE_ID  | 40006 |
