# details about LEB_FED software
The basics of the LED_FEB software is based on a lightweight
implementation of modbus called 
[liblightmodbus](https://github.com/Jacajack/liblightmodbus).
It is a header only implementation, which minimizes installation sizes.

Most logic is handled by the modbus library.  Implemenation specific
differences are limited to a few places. If hardware registers need to
be set, they are set in case statements within the modbus framework.
The only exception being the power on/off functions, because they
require a relatively long time to process, which can cause
timeouts in the modbus framework.

## setting SLAVE_ID
One of the first things to do for a new LED_FEB is to set its final SLAVE_ID
for modbus. By default, the SLAVE_ID is set to 20. It needs to be set to
it's real SLAVE_ID, which then is stored in the EEPROM of the MCU.

There are two ways to archieve this:
 - if SLAVE_ID is 20, writing to modbus HOLDING register at 0x0 will set
   a new SLAVE_ID.
 - writing to HOLDING register LIGHTMODBUS_SLAVE_ID will set it regardless
   of the old SLAVE_ID
Both methods will update the value in the EEPROM.

## prepare for flashing LEDs
### Sequence to flash LEDs
The sequence for having LEDs flash is the following:

1. [Power on LED-FEB](#power-on-led_feb)
1. [Wait for POWER ready](#power-status)
1. [Select Trigger source](#trigger-source)
1. [Trigger enable](#trigger-enable)
1. [Select LED](#select-led)
1. [Set LED_BIAS voltage](#set-led_bias)
1. [Enable LED_BIAS](#enable-led_bias)

### Power off Sequence
The sequence for stop flashing the LEDs is the following:

1. [Power off LED-FEB](#power-off-led_feb)
1. [Set Trigger source to 0](#trigger-source)
1. [Trigger disable](#trigger-enable)

## Description of all ModBus Functions
The following ModBus functions are available:

### Power on LED_FEB
Write 1 to ModBus Coil to register POWER_ON to power on.
It will also switch all LEDs off and set the LED_BIAS to its minimun (0xFFF).
Use ModBus Coil FIRST_PULSE_FIX to apply/not apply a fix for the first pulse issue.<br>
Will set [Power Status](#power-status) at end of sequence.

### Power off LED_FEB
Write 0 to ModBus Coil to register POWER_ON to power off.
It will also switch all LEDs off and set the LED_BIAS to its minimun (0xFFF).<br>
Will set [Power Status](#power-status) at end of sequence.

### Power Status
Enquire the power status of the board.<br>
Read from ModBus Discrete Input POWER_STATUS, 0 means off or during power up,
1 means fully power up.

### Select Trigger source
Write to ModBus Holding Register TRIGGER_SOURCE, 0 means board, 1 means internal 2 means external.
Writing anything else will increase Error count in ModBus Input Register TRIG_SOURCE_ERR.
If using 1 as trigger source, ModBus Holding Register uC_FLASH_FREQ can be used to modify the
[flashing frequency](#modify-flashing-frequency).

### Trigger enable
Write 1 to ModBus Coil to register TRIGGER_ENABLE to enable, write 0 to disable.

### Select LED
Currently a bit pattern with bits 0-6 used to enable/disable respective LEDs.
Multiple LEDs can flash at a time. Unused bits are ignored. The monitoring
array for the IMON will be set to all zeros.
Write bitpattern to ModBus Holding REgister LED_CHANNELS.

### Set LED_BIAS
Write 14 bit into DAC to set the LED_BIAS voltage. ModBus accepts hex number,
it's up to the user to convert from a desired voltage.
Write bitpattern into ModBus Holding Register LED_BIAS_DAC_SET.

### Enable LED_BIAS
Write 1 to ModBus Coil LED_BIAS_ENABLE to enable, write 0 to disable.

## Possible Tests during flashing
There are some checks possible that can be used to make sure all values are within
reasonabe range. 

Two ADCs are reading out constantly voltage and current for the LEDs. The frequency is
hardcoded to 40Hz, via corresponding calls to the function osDelay(25). The next two
function calls then just return the latest values.
Failures to read those values are counted in the ModBus Holding Register LED_BIAS_ERROR.

### Read back LED_BIAS voltage
The preset LED_BIAS voltage can be read back with a 12bit ADC. Note that the translation
from ADC counts to voltage differs from the one set in the
ModBus Holding Register LED_BIAS_DAC_SET.
This ModBus function doesn't initiate a new readout of the ADC, but returns the latest
value read at 40Hz.
Read 12 bit from ModBus Holding Register LED_BIAS_ADC_READ.

### Read back IMON current
This reads back the current that the LED flashing circuit is consuming.
This ModBus function doesn't initiate a new readout of the ADC, but returns the latest
value read at 40Hz.
Read 12 bit from ModBus Holding Register IMON_ADC_READ.

### detect LED_Current too high
At every readout of the two ADC values for LED voltage and current, it is tested if the
average over the last 5 current measurements exceeds a configurable threshold, set in
ModBus Holding Register ALARM_THRESHOLD. If the threshold is exceeded, an error counter
is incremented.
Read the error counter from ModBus Holding Register LED_CURRENT_ERROR.

### Set the Alarm Threshold for LED_Current
The alarm threshold in ADC counts can be set by writing to Holding Register ALARM_THRESHOLD.

## Read out Error Counters
Five Error counters are implemented:
Each of them can be read out by reading from the corersponding ModBus Input Register.

### LED_CURRENT_ERROR
see the [description](#detect-led_current-too-high) for details.

### LED_BIAS_ERROR
see the [description](#read-back-LED_BIAS-voltage) for details.

### TRIG_SOURCE_ERR
Writing anything different from 0, 1 or 2 into ModBus holding register TRIGGER_SOURCE
will increase Error count in ModBus Input Register TRIG_SOURCE_ERR.

### LED_FEB_MB_SLAVE_ERR
Errors in the processing of ModBus requests inside liblightmodbus calls increase these counters.

### LED_FEB_MB_GLOBAL_ERR
This counter logs serious errors inside liblightmodbus like memory allocation errors in the
microcontroller.

## Miscellanious ModBus Functions
Following miscellanious ModBus functions are available:

### firmware version
Read from ModBus Holding Register FIRMWARE_VERSION, the lower 3 nibbles represent the
firmware version:<br>
0x0MNP<br>
with<br>
M == mayor version<br>
N == minor version<br>
P == patch level<br>

### Modify Flashing Frequency
When in internal trigger mode, the flashing frequency for the LEDs needs to be
configured otherwise they won't flash. This is controlled by writing a control word W
into into ModBus Holding Register uC_FLASH_FREQ.

F=32MHz/2 / (W+1)

Note, that the value will only be written if W>0.

### Reset Error Count
To reset all error counts, write 1 into  ModBus Coil RESET_ERROR_COUNT

