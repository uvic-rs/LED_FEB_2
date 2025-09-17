import minimalmodbus
import time

# power on         "POWER ON"
# set dac at 2000  (to be tested)
# trig soft        "TRIGGER_SOURCE"
# pulses on        "TRIGGER_ENABLE"
# select 0 (LED)   "LED_CHANNELS"
# enable           

# MODBUS function nr                                R        W
# Read-and-write bits (coils) 	                   1        5
# Read-only bits (discrete inputs) 	  	   2        None
# Read-and-write registers (holding registers) 	   3, 23    6
# Read-only registers (input registers) 	   4, 23    None

PWR_ON          =     1
TRIGGER_ENABLE  =     2
BIAS_ENABLE     =     3

PWR_STATUS      = 10001

FIRMWARE_VERSION= 30008

TRIGGER_SOURCE  = 40001
LED_CHANNELS    = 40002
LED_DAC_BIAS    = 40003
LED_BIAS_ADC_READ=40004
MODBUS_SLAVE_ID = 40007

LEDFEB_ID=25
old_ID=23

LEDindex=2
LED=1<<LEDindex

# set LEDFEB_ID to new value
test_board_20 = minimalmodbus.Instrument('/dev/ttyUSB0', old_ID)
test_board_20.serial.baudrate=115200
test_board_20.mode = minimalmodbus.MODE_RTU
test_board_20.serial.timeout  = 0.1
test_board_20.close_port_after_each_call = True
try:
    test_board_20.write_register(MODBUS_SLAVE_ID, LEDFEB_ID, functioncode=6)
except:
    print("Cannot talk to LEDFEB with ID=", old_ID) 
time.sleep(0.1)
time.sleep(5)

# test_board = minimalmodbus.Instrument('/dev/ttyACM0', LEDFEB_ID, debug=True)
test_board = minimalmodbus.Instrument('/dev/ttyUSB0', LEDFEB_ID)
test_board.serial.baudrate=115200
test_board.mode = minimalmodbus.MODE_RTU
test_board.serial.timeout  = 0.1
test_board.close_port_after_each_call = True

print(test_board)

print("r POWER_ON ", test_board.read_bit(PWR_ON, 1))
time.sleep(0.1)
print("r TRIG EN  ", test_board.read_bit(TRIGGER_ENABLE, 1))
time.sleep(0.1)

time.sleep(1.0)
print("powering on")
test_board.write_bit(PWR_ON, 1, functioncode=5)
for i in range(10):
    print("r POWER_STATUS ", test_board.read_bit(PWR_STATUS, 2))
time.sleep(1.5)
for i in range(10):
    print("r POWER_STATUS ", test_board.read_bit(PWR_STATUS, 2))
print("trigger source to '1'")
test_board.write_register(TRIGGER_SOURCE, 1, functioncode=6)
time.sleep(0.1)
print("trigger enable")
test_board.write_bit(TRIGGER_ENABLE, 1, functioncode=5)
time.sleep(0.1)

print("LED_BIAS to 0")
test_board.write_register(LED_DAC_BIAS, 0, functioncode=6)
time.sleep(0.1)
print("LED_BIAS enable")
test_board.write_bit(BIAS_ENABLE, 1, functioncode=5)
time.sleep(0.1)

for round in range(10):
    for li in range(0,7):
        LED=1<<li
        print("LED %d (bit %d) on" % (li, LED))
        test_board.write_register(LED_CHANNELS, LED, functioncode=6)
        time.sleep(0.1)
    
time.sleep(0.25)
test_board.write_register(LED_CHANNELS, 127, functioncode=6)
    
for i in range(0,4095,64):
    print(" %4d  : " % i, end="")
    test_board.write_register(LED_DAC_BIAS, i, functioncode=6)
    time.sleep(0.125)
    adcv=test_board.read_register(LED_BIAS_ADC_READ, 0, functioncode=3)
    print(hex(adcv), end="  ")
    ibias=test_board.read_register(LED_BIAS_ADC_READ+1, 0, functioncode=3)
    print(hex(ibias), end=" >> ")
    v1=15.3-13.6/4096*i
    v2=16.5*adcv/4096
    ib=3.3*ibias/4096/200 * 1000
    print(" %6.3f V : %6.3f V   %6.3f mA" % (v1, v2, ib))

test_board.write_register(LED_DAC_BIAS, 0, functioncode=6)
print("r POWER_STATUS ", test_board.read_bit(PWR_STATUS, 2))
print("cycle s")
for li in range(0,7):
    LED=1<<li
    print("c LED %d (bit %d) on" % (li, LED))
    test_board.write_register(LED_CHANNELS, LED, functioncode=6)
    time.sleep(0.1)
    for i in range(0,4095,64):
        print(" %4d  : " % i, end="")
        test_board.write_register(LED_DAC_BIAS, i, functioncode=6)
        #  time.sleep(0.125)
        adcv=test_board.read_register(LED_BIAS_ADC_READ, 0, functioncode=3)
        print(hex(adcv), end="  ")
        ibias=test_board.read_register(LED_BIAS_ADC_READ+1, 0, functioncode=3)
        print(hex(ibias), end=" >> ")
        v1=15.3-13.6/4096*i
        v2=16.5*adcv/4096
        ib=3.3*ibias/4096/200 * 1000
        print(" %6.3f V : %6.3f V   %6.3f mA" % (v1, v2, ib))

print("cycle S")



print("Firmware version : ", end="")
print(hex(test_board.read_register(FIRMWARE_VERSION, 0, functioncode=4)))
print("ERR LED Curr     : ", end="")
print(test_board.read_register(30002, 0, functioncode=4))
print("ERR LED Bias     : ", end="")
print(test_board.read_register(30003, 0, functioncode=4))
print("ERR Trg Source   : ", end="")
print(test_board.read_register(30004, 0, functioncode=4))
print("ERR MB Slave     : ", end="")
print(test_board.read_register(30005, 0, functioncode=4))
print("ERR MB global    : ", end="")
print(test_board.read_register(30006, 0, functioncode=4))
print("rest - index used internally : ")
print(test_board.read_register(30012, 0, functioncode=4))
print(test_board.read_register(30013, 0, functioncode=4))
print(test_board.read_register(30014, 0, functioncode=4))
print(test_board.read_register(30015, 0, functioncode=4))

# print(register_1)
# print(register_2)

test_board.write_register(LED_CHANNELS, 0, functioncode=6)

time.sleep(0.1)
print("powering off")
test_board.write_bit(PWR_ON, 0, functioncode=5)
time.sleep(0.5)
print("trigger source to '0'")
test_board.write_register(TRIGGER_SOURCE, 0, functioncode=6)
time.sleep(0.1)

print("LED_BIAS to 0")
test_board.write_register(LED_DAC_BIAS, 4095, functioncode=6)
time.sleep(0.1)

print("trigger disable")
test_board.write_bit(TRIGGER_ENABLE, 0, functioncode=5)


time.sleep(0.1)
print("r TRIG EN  ", test_board.read_bit(TRIGGER_ENABLE, 1))
time.sleep(0.1)
print("r POWER_ON ", test_board.read_bit(PWR_ON, 1))

print("r POWER_STATUS ", test_board.read_bit(PWR_STATUS, 2))


