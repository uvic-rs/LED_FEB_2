# LED_FEB_2
firmware for the LED_FEB in the Hyper-K experiment. More [details](SWdetails.md)
can be found here.

## The Definitions for the various ModBus register types and registers
### Coils
| function | Modbus addresses | comment |
| ------------- |:-------------:| ------------- |
| POWER_ON         |          1 | flag to power on/off board |
| TRIGGER_ENABLE   |          2 | TRIGGER enable/disable |
| LED_BIAS_ENABLE  |          3 | on/off LED_BIAS |
| FIRST_PULSE_FIX  |          4 | flag to apply fix first pulse issue |
| RESET_ERROR_COUNT |         5 | reset error counts |

### Discrete Inputs
| function | Modbus addresses | comment |
| ------------- |:-------------:| ------------- |
| POWER_STATUS |        10001 | return power status |

### Input Registers
| function | Modbus addresses | comment |
| ------------- |:-------------:| ------------- |
| FIRMWARE_VERSION      | 30001 | return firmware version |
| LED_BIAS_ADC_READ     | 30002 | return LED_BIAS adc count |
| IMON_ADC_READ         | 30003 | return I_mon adc count |
| LED_CURRENT_ERROR     | 30004 | LED current above threshold |
| LED_BIAS_ERROR        | 30005 | ADC readout errors |
| TRIG_SOURCE_ERR       | 30006 | wrong trigger selected |
| LED_FEB_MB_SLAVE_ERR  | 30007 | modbus slave errors |
| LED_FEB_MB_GLOBAL_ERR | 30008 | modbus global errors |

### Holding Registers
| function | Modbus addresses | comment |
| ------------- |:-------------:| ------------- |
| TRIGGER_SOURCE        | 40001 | select trigger source 0: board 1: uC 2: external |
| LED_CHANNELS          | 40002 | select which LED channel to flash (bit) |
| LED_BIAS_DAC_SET      | 40003 | select LED BIAS in DAC counts |
| ALARM_THRESHOLD       | 40004 | threshold for LED_CURRENT_ERROR |
| uC_FLASH_FREQ         | 40005 | LED flash frequency for uC as trigger f=16MHz/n (n!=0) |
| LIGHTMODBUS_SLAVE_ID  | 40006 | ID for modbus slave |

ChangeLog:

- Oct. 14, 2025 - Rolf S - ver. 0.2.0: add some safety checks
  - constantly monitor LED current and increase error count if above
    average over last 5 samples larger than ALARM_THRESHOLD (configurable)
  - make LED flash frequency configurable, off if set to 0
- Sep. 16, 2025 - Rolf S - ver. 0.1.1: cleanup and imported to github
- Aug. 10, 2025 - Rolf S - ver. 0.1.0: ported to new hardware, M3