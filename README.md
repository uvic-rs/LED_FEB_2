# LED_FEB_2
firmware for the LED_FEB in the Hyper-K experiment.

##Coils
| function | Modbus addresses |
| POWER_ON         |          1 |
| TRIGGER_ENABLE   |          2 |
| LED_BIAS_ENABLE  |          3 |


## Discrete Inputs
| function | Modbus addresses |
| POWER_STATUS |        10001 |

## Input Registers
| function | Modbus addresses |
| R_FIRMWARE_VERSION      | 0 |
| R_LED_CURRENT_ERROR     | 1 |
| R_LED_BIAS_ERROR        | 2 |
| R_TRIG_SOURCE_ERR       | 3 |
| R_LED_FEB_MB_SLAVE_ERR  | 4 |
| R_LED_FEB_MB_GLOBAL_ERR | 5 |

## Holding Registers
| function | Modbus addresses |
| O_TRIGGER_SOURCE        | 0 |
| O_LED_CHANNELS          | 1 |
| O_LED_BIAS_DAC_SET      | 2 |
| O_LED_BIAS_ADC_READ     | 3 |
| O_IMON_ADC_READ         | 4 |
| O_LIGHTMODBUS_SLAVE_ID  | 5 |
