# Version 1.1 - R. Gornea on October 8, 2025 - uograde partially based on Rolf S. updates, added support for pymodbus 3.9.2
# Version 1.0 - R. Gornea on August 31, 2025 - initial support Mach III LED-FEB running MODBUS firmware 0.1.0 or greater
# Version 0.3 - R. Gornea on July 17, 2025 - added support for error detection, autosetting DAC and LED number
# Version 0.2 - R. Gornea on July 2, 2025 - added support for automatic LED-FEB indexing on main board
# Version 0.1 - R. Gornea on June 25, 2025 - basic support for Mach II LED-FEB running MODBUS firmware

import time
import sys
import os
from reg_map import Basic_RW
from pymodbus.client import ModbusTcpClient
from pymodbus import ModbusException

# MODBUS coils
PWR_ON = 1
TRIGGER_ENABLE = 2
LED_BIAS_ENABLE = 3
# MODBUS discrete inputs
POWER_STATUS = 10001
# MODBUS input registers
FIRMWARE_VERSION  = 30001
LED_CURRENT_ERROR = 30002
LED_BIAS_ERROR = 30003
TRIG_SOURCE_ERR = 30004
LED_FEB_MB_SLAVE_ERR = 30005
LED_FEB_MB_GLOBAL_ERR = 30006
# MODBUS registers
TRIGGER_SOURCE = 40001
LED_CHANNELS = 40002
LED_DAC_BIAS = 40003
LED_BIAS_ADC = 40004
IMON_ADC = 40005
MODBUS_SLAVE_ID = 40006
# Constants
BASE_MMAP_ENCODING_ADDR = 85
SELECT_MAIN_BOARD = 0x0
SELECT_EXT_CRYSTAL = 0x1
SELECT_uCONTROLLER = 0x2
POWER_ENABLE_REG_ADDR = 1
IS_PMT_REG_ADDR = 64
MODBUS_ADDR_SEQUENCE = [21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39]

class xFEB_Modbus:
    def __init__(self, debug_modbus_addr = 20):
        self.n_febs = 0
        self.febs_array = []
        self.n_led_febs = 0
        self.led_febs_array = []
        self.run_mode = False # flags the ability to open the run control register file
        self.connected = False # flags the MODBUS connection status 
        if (os.path.exists("/dev/uio0") == True):
            self.run_ctrl = Basic_RW() # open access to the memory mapped registers
            self.run_mode = True # the call was successful
            # scan for the number of LED-FEBs the system has available
            led_feb_available = ~self.run_ctrl.ReadReg(IS_PMT_REG_ADDR) & 0x7FFFF
            xfebs_powered = self.run_ctrl.ReadReg(POWER_ENABLE_REG_ADDR) & 0x7FFFF
            # print(f"LED available are {led_feb_available} and LED powered are {xfebs_powered}")
            led_feb_available &= xfebs_powered
            for my_index in range(19):
                if (led_feb_available & (0x1 << my_index)):
                    self.led_febs_array.append(MODBUS_ADDR_SEQUENCE[self.n_led_febs])
                    self.n_led_febs += 1
        else:
            print(f"[WARNING]: Runing with a single LED-FEB board at MODBUS address {debug_modbus_addr}")
            self.n_led_febs = 1
            self.led_febs_array.append(debug_modbus_addr)
        self.modbus_client = ModbusTcpClient("localhost")
        self.modbus_client.connect()
        if (self.modbus_client.connected == True):
            self.connected = True
        else:
            print("[ERROR]: Could not connect to the MODBUS device")
            sys.exit(-2)
    
    # low level function call
    def is_connected(self):
        return self.connected
    
    # high level function call
    def get_n_febs(self):
        return self.n_febs
    
    # low level function call
    def get_feb_addr(self, feb_id):
        if (feb_id < self.n_febs):
            return self.febs_array[feb_id]
        else:
            return None
    
    # high level function call
    def get_n_led_febs(self):
        return self.n_led_febs
    
    # low level function call
    def get_led_feb_addr(self, led_feb_id):
        if (led_feb_id < self.n_led_febs):
            return self.led_febs_array[led_feb_id]
        else:
            return None
    
    # low level function call
    def set_register(self, modbus_id, reg_addr, reg_value):
        if (self.connected):
            if (self.run_mode == False):
                if (self.led_febs_array[0] != modbus_id):
                    print(f"[WARNING]: Ignored requested MODBUS ID {modbus_id} and used opened device ID {self.led_febs_array[0]}")
                try:
                    my_reply = self.modbus_client.write_register(reg_addr, reg_value, slave = self.led_febs_array[0])
                except ModbusException as my_exception:
                    print(f"[ERROR]: pymodbus exception {my_exception} has occured in set_register function")
                    return False
                if (my_reply.isError()):
                    print("[ERROR]: an error has been returned by the debugging LED-FEB")
                    return False
            else:
                try:
                    my_reply = self.modbus_client.write_register(reg_addr, reg_value, slave = modbus_id)
                except ModbusException as my_exception:
                    print(f"[ERROR]: pymodbus exception {my_exception} has occured in set_register function")
                    return False
                if (my_reply.isError()):
                    print(f"[ERROR]: an error has been returned by the LED-FEB with ID {modbus_id}")
                    return False 
            return True
        else:
            print("[ERROR]: no connection has been established")
            return False
    
    # low level function call
    def get_registers(self, modbus_id, start_addr, my_count):
        if (self.connected):
            if (self.run_mode == False):
                if (self.led_febs_array[0] != modbus_id):
                    print(f"[WARNING]: Ignored requested MODBUS ID {modbus_id} and used opened device ID {self.led_febs_array[0]}")
                try:
                    my_reply = self.modbus_client.read_holding_registers(start_addr, count = my_count, slave = self.led_febs_array[0])
                except ModbusException as my_exception:
                    print(f"[ERROR]: pymodbus exception {my_exception} has occured in get_registers function")
                    return None
                if (my_reply.isError()):
                    print("[ERROR]: an error has been returned by the debugging LED-FEB")
                    return None
                return my_reply
            else:
                try:
                    my_reply = self.modbus_client.read_holding_registers(start_addr, count = my_count, slave = modbus_id)
                except ModbusException as my_exception:
                    print(f"[ERROR]: pymodbus exception {my_exception} has occured in get_registers function")
                    return None
                if (my_reply.isError()):
                    print(f"[ERROR]: an error has been returned by the LED-FEB with ID {modbus_id}")
                    return None
                return my_reply
        else:
            print("[ERROR]: no connection has been established")
            return None

    # low level function call
    def get_register(self, modbus_id, reg_addr):
        temp_read = self.get_registers(modbus_id, reg_addr, 1)
        if (temp_read == None):
            return temp_read
        else:
            return temp_read.registers[0]

    # low level function call
    def get_inputs(self, modbus_id, start_addr, my_count):
        if (self.connected):
            if (self.run_mode == False):
                if (self.led_febs_array[0] != modbus_id):
                    print(f"[WARNING]: Ignored requested MODBUS ID {modbus_id} and used opened device ID {self.led_febs_array[0]}")
                try:
                    my_reply = self.modbus_client.read_input_registers(start_addr, count = my_count, slave = self.led_febs_array[0])
                except ModbusException as my_exception:
                    print(f"[ERROR]: pymodbus exception {my_exception} has occured in get_inputs function")
                    return None
                if (my_reply.isError()):
                    print("[ERROR]: an error has been returned by the debugging LED-FEB")
                    return None
                return my_reply
            else:
                try:
                    my_reply = self.modbus_client.read_input_registers(start_addr, count = my_count, slave = modbus_id)
                except ModbusException as my_exception:
                    print(f"[ERROR]: pymodbus exception {my_exception} has occured in get_inputs function")
                    return None
                if (my_reply.isError()):
                    print(f"[ERROR]: an error has been returned by the LED-FEB with ID {modbus_id}")
                    return None
                return my_reply
        else:
            print("[ERROR]: no connection has been established")
            return None

    # low level function call
    def get_input(self, modbus_id, reg_addr):
        temp_read = self.get_inputs(modbus_id, reg_addr, 1)
        if (temp_read == None):
            return temp_read
        else:
            return temp_read.registers[0]
  
    # low level function call
    def set_coil(self, modbus_id, coil_addr, coil_bool):
        if (self.connected):
            if (self.run_mode == False):
                if (self.led_febs_array[0] != modbus_id):
                    print(f"[WARNING]: Ignored requested MODBUS ID {modbus_id} and used opened device ID {self.led_febs_array[0]}")
                try:
                    # my_reply = self.modbus_client.write_coil(coil_addr, coil_bool, self.led_febs_array[0])
                    my_reply = self.modbus_client.write_coil(coil_addr, coil_bool, slave = self.led_febs_array[0])
                except ModbusException as my_exception:
                    print(f"[ERROR]: pymodbus exception {my_exception} has occured in set_coil function")
                    return False
                if (my_reply.isError()):
                    print("[ERROR]: an error has been returned by the debugging LED-FEB")
                    return False
            else:
                try:
                    my_reply = self.modbus_client.write_coil(coil_addr, coil_bool, slave = modbus_id)
                except ModbusException as my_exception:
                    print(f"[ERROR]: pymodbus exception {my_exception} has occured in set_coil function")
                    return False
                if (my_reply.isError()):
                    print(f"[ERROR]: an error has been returned by the LED-FEB with ID {modbus_id}")
                    return False
            return True
        else:
            print("[ERROR]: no connection has been established")
            return False
    
    # low level function call
    def get_coils(self, modbus_id, start_addr, my_count):
        if (self.connected):
            if (self.run_mode == False):
                if (self.led_febs_array[0] != modbus_id):
                    print(f"[WARNING]: Ignored requested MODBUS ID {modbus_id} and used opened device ID {self.led_febs_array[0]}")
                try:
                    my_reply = self.modbus_client.read_coils(start_addr, count = my_count, slave = self.led_febs_array[0])
                except ModbusException as my_exception:
                    print(f"[ERROR]: pymodbus exception {my_exception} has occured in get_coils function")
                    return None
                if (my_reply.isError()):
                    print("[ERROR]: an error has been returned by the debugging LED-FEB")
                    return None
                return my_reply
            else:
                try:
                    my_reply = self.modbus_client.read_coils(start_addr, count = my_count, slave = modbus_id)
                except ModbusException as my_exception:
                    print(f"[ERROR]: pymodbus exception {my_exception} has occured in get_coils function")
                    return None
                if (my_reply.isError()):
                    print(f"[ERROR]: an error has been returned by the LED-FEB with ID {modbus_id}")
                    return None
                return my_reply
        else:
            print("[ERROR]: no connection has been established")
            return None   
    
    # low level function call
    def get_coil(self, modbus_id, coil_addr):
        temp_read = self.get_coils(modbus_id, coil_addr, 1)
        if (temp_read == None):
            return temp_read
        else:
            return temp_read.bits[0]

    # low level function call
    def get_discrete_inputs(self, modbus_id, start_addr, my_count):
        if (self.connected):
            if (self.run_mode == False):
                if (self.led_febs_array[0] != modbus_id):
                    print(f"[WARNING]: Ignored requested MODBUS ID {modbus_id} and used opened device ID {self.led_febs_array[0]}")
                try:
                    my_reply = self.modbus_client.read_discrete_inputs(start_addr, count = my_count, slave = self.led_febs_array[0])
                except ModbusException as my_exception:
                    print(f"[ERROR]: pymodbus exception {my_exception} has occured in get_discretes function")
                    return None
                if (my_reply.isError()):
                    print("[ERROR]: an error has been returned by the debugging LED-FEB")
                    return None
                return my_reply
            else:
                try:
                    my_reply = self.modbus_client.read_discrete_inputs(start_addr, count = my_count, slave = modbus_id)
                except ModbusException as my_exception:
                    print(f"[ERROR]: pymodbus exception {my_exception} has occured in get_discretes function")
                    return None
                if (my_reply.isError()):
                    print(f"[ERROR]: an error has been returned by the LED-FEB with ID {modbus_id}")
                    return None
                return my_reply
        else:
            print("[ERROR]: no connection has been established")
            return None   
    
    # low level function call
    def get_discrete(self, modbus_id, coil_addr):
        temp_read = self.get_discrete_inputs(modbus_id, coil_addr, 1)
        if (temp_read == None):
            return temp_read
        else:
            return temp_read.bits[0]

    # high level function call
    def power_on(self, led_feb_id):
        if (led_feb_id < self.get_n_led_febs()):
            my_status = self.set_coil(self.get_led_feb_addr(led_feb_id), PWR_ON, True)
            if (my_status == False):
                print(f"[EXIT]: Failed to power on LED-FEB {led_feb_id}")
                sys.exit(-2)
            else:
                return my_status
        else:
            return False

    # high level function call
    def power_off(self, led_feb_id):
        if (led_feb_id < self.get_n_led_febs()):
            return self.set_coil(self.get_led_feb_addr(led_feb_id), PWR_ON, False)
        else:
            return False

    # high level function call
    def power_status(self, led_feb_id):
        if (led_feb_id < self.get_n_led_febs()):
            return self.get_discrete(self.get_led_feb_addr(led_feb_id), POWER_STATUS)
        else:
            return None

    # high level function call
    def trig_enable(self, led_feb_id):
        if (led_feb_id < self.get_n_led_febs()):
            return self.set_coil(self.get_led_feb_addr(led_feb_id), TRIGGER_ENABLE, True)
        else:
            return False

    # high level function call
    def trig_disable(self, led_feb_id):
        if (led_feb_id < self.get_n_led_febs()):
            return self.set_coil(self.get_led_feb_addr(led_feb_id), TRIGGER_ENABLE, False)
        else:
            return False

    # high level function call
    def trig_status(self, led_feb_id):
        if (led_feb_id < self.get_n_led_febs()):
            return self.get_coil(self.get_led_feb_addr(led_feb_id), TRIGGER_ENABLE)
        else:
            return None

    # high level function call
    def led_bias_enable(self, led_feb_id):
        if (led_feb_id < self.get_n_led_febs()):
            return self.set_coil(self.get_led_feb_addr(led_feb_id), LED_BIAS_ENABLE, True)
        else:
            return False

    # high level function call
    def led_bias_disable(self, led_feb_id):
        if (led_feb_id < self.get_n_led_febs()):
            return self.set_coil(self.get_led_feb_addr(led_feb_id), LED_BIAS_ENABLE, False)
        else:
            return False

    # high level function call
    def led_bias_status(self, led_feb_id):
        if (led_feb_id < self.get_n_led_febs()):
            return self.get_coil(self.get_led_feb_addr(led_feb_id), LED_BIAS_ENABLE)
        else:
            return False

    # high level function call
    def set_led_bias(self, led_feb_id, level_in_volts):
        if (led_feb_id < self.get_n_led_febs()):
            dac_level = int(4887.2 - 325.66 * level_in_volts)
            if dac_level > 4095:
                dac_level = 4095
            elif dac_level < 0:
                dac_level = 0
            my_reply = self.set_register(self.get_led_feb_addr(led_feb_id), LED_DAC_BIAS, dac_level)
            if (my_reply == True):
                if (self.run_mode == True):
                    my_register_value = self.run_ctrl.ReadReg(BASE_MMAP_ENCODING_ADDR + led_feb_id) & 0x7F000
                    my_register_value |= (dac_level & 0xFFF)
                    self.run_ctrl.WriteReg(BASE_MMAP_ENCODING_ADDR + led_feb_id, my_register_value)
                    # print(f"Wrote value {my_register_value} at register {BASE_MMAP_ENCODING_ADDR + led_feb_id}")
            return my_reply
        else:
            return False

    # high level function call
    def set_max_led_bias(self, led_feb_id):
        return self.set_led_bias(led_feb_id, 15.1)

    # high level function call
    def set_min_led_bias(self, led_feb_id):
        return self.set_led_bias(led_feb_id, 2.3)

    # low level function call
    def get_led_bias(self, led_feb_id):
        if (led_feb_id < self.get_n_led_febs()):
            adc_level = self.get_register(self.get_led_feb_addr(led_feb_id), LED_BIAS_ADC) & 0xFFF
            level_in_volts = 16.5 * adc_level / 4096
            return level_in_volts
        else:
            return None

    # high level function call
    def fetch_led_bias(self, led_feb_id):
        value = self.get_led_bias(led_feb_id)
        if (value == None):
            return "Invalid LED ID request or bias value"
        elif (value < 2.4):
            return "LED bias voltage is set to the minimum"
        elif (value > 15.0):
            return "LED bias voltage is set to the maximum"
        else:
            return f"LED bias voltage is set to {value} volts"

    # low level function call
    def get_i_mon(self, led_feb_id):
        if (led_feb_id < self.get_n_led_febs()):
            adc_level = self.get_register(self.get_led_feb_addr(led_feb_id), IMON_ADC) & 0xFFF
            level_in_mA = 3300 * adc_level / 4096 / 200
            return level_in_mA
        else:
            return None

    # high level function call
    def fetch_i_mon(self, led_feb_id):
        value = self.get_i_mon(led_feb_id)
        if (value == None):
            return "Invalid LED ID request or current monitor value"
        else:
            return f"LED bias current is {value} mA"  
  
    # low level function call
    def set_channels(self, led_feb_id, channel_pattern):
        if (led_feb_id < self.get_n_led_febs()):
            my_reply = self.set_register(self.get_led_feb_addr(led_feb_id), LED_CHANNELS, channel_pattern & 0x7F)
            if (my_reply == True):
                if (self.run_mode == True):
                    my_register_value = self.run_ctrl.ReadReg(BASE_MMAP_ENCODING_ADDR + led_feb_id) & 0xFFF
                    my_register_value |= ((channel_pattern & 0x7F) << 12)
                    self.run_ctrl.WriteReg(BASE_MMAP_ENCODING_ADDR + led_feb_id, my_register_value)
            return my_reply
        else:
            return False

    # low level function call
    def get_channels(self, led_feb_id):
        if (led_feb_id < self.get_n_led_febs()):
            return self.get_register(self.get_led_feb_addr(led_feb_id), LED_CHANNELS)
        else:
            return None
    
    # low level function call
    def selective_set_channels(self, led_feb_id, channel_pattern):
        if (led_feb_id < self.get_n_led_febs()):
            current_channel_pattern = self.get_channels(led_feb_id)
            return self.set_channels(led_feb_id, current_channel_pattern | channel_pattern)
        else:
            return False

    # low level function call
    def selective_clear_channels(self, led_feb_id, channel_pattern):
        if (led_feb_id < self.get_n_led_febs()):
            current_channel_pattern = self.get_channels(led_feb_id)
            return self.set_channels(led_feb_id, current_channel_pattern & (~channel_pattern & 0x7F))
        else:
            return False

    # high level function call
    def enable_channel_0(self, led_feb_id):
        return self.selective_set_channels(led_feb_id, 0x01)

    # high level function call
    def disable_channel_0(self, led_feb_id):
        return self.selective_clear_channels(led_feb_id, 0x01)

    # high level function call
    def set_channel_0(self, led_feb_id):
        return self.set_channels(led_feb_id, 0x01)

    # high level function call
    def enable_channel_1(self, led_feb_id):
        return self.selective_set_channels(led_feb_id, 0x02)

    # high level function call
    def disable_channel_1(self, led_feb_id):
        return self.selective_clear_channels(led_feb_id, 0x02)

    # high level function call
    def set_channel_1(self, led_feb_id):
        return self.set_channels(led_feb_id, 0x02)
    
    # high level function call
    def enable_channel_2(self, led_feb_id):
        return self.selective_set_channels(led_feb_id, 0x04)

    # high level function call
    def disable_channel_2(self, led_feb_id):
        return self.selective_clear_channels(led_feb_id, 0x04)

    # high level function call
    def set_channel_2(self, led_feb_id):
        return self.set_channels(led_feb_id, 0x04)
    
    # high level function call
    def enable_channel_3(self, led_feb_id):
        return self.selective_set_channels(led_feb_id, 0x08)

    # high level function call
    def disable_channel_3(self, led_feb_id):
        return self.selective_clear_channels(led_feb_id, 0x08)

    # high level function call
    def set_channel_3(self, led_feb_id):
        return self.set_channels(led_feb_id, 0x08)

    # high level function call
    def enable_channel_4(self, led_feb_id):
        return self.selective_set_channels(led_feb_id, 0x10)

    # high level function call
    def disable_channel_4(self, led_feb_id):
        return self.selective_clear_channels(led_feb_id, 0x10)

    # high level function call
    def set_channel_4(self, led_feb_id):
        return self.set_channels(led_feb_id, 0x10)

    # high level function call
    def enable_channel_5(self, led_feb_id):
        return self.selective_set_channels(led_feb_id, 0x20)

    # high level function call
    def disable_channel_5(self, led_feb_id):
        return self.selective_clear_channels(led_feb_id, 0x20)

    # high level function call
    def set_channel_5(self, led_feb_id):
        return self.set_channels(led_feb_id, 0x20)
    
    # high level function call
    def enable_channel_6(self, led_feb_id):
        return self.selective_set_channels(led_feb_id, 0x40)

    # high level function call
    def disable_channel_6(self, led_feb_id):
        return self.selective_clear_channels(led_feb_id, 0x40)

    # high level function call
    def set_channel_6(self, led_feb_id):
        return self.set_channels(led_feb_id, 0x40)
    
    # high level function call
    def enable_all_channels(self, led_feb_id):
        return self.set_channels(led_feb_id, 0x7F)

    # high level function call
    def disable_all_channels(self, led_feb_id):
        return self.set_channels(led_feb_id, 0x00)

    # low level function call
    def set_trig_source(self, led_feb_id, trig_source):
        if (led_feb_id < self.get_n_led_febs()):
            return self.set_register(self.get_led_feb_addr(led_feb_id), TRIGGER_SOURCE, trig_source)
        else:
            return False

    # high level function call
    def fetch_trig_source(self, led_feb_id):
        if (led_feb_id < self.get_n_led_febs()):
            match self.get_register(self.get_led_feb_addr(led_feb_id), TRIGGER_SOURCE):
                case int(SELECT_MAIN_BOARD):
                    return "Pulses are triggered by the main board"
                case int(SELECT_EXT_CRYSTAL):
                    return "Pulses are triggered by the external crystal"
                case int(SELECT_uCONTROLLER):
                    return "Pulses are triggered by the microcontroller"
                case _:
                    return "[WARNING]: Unknown trigger source selected"
        else:
            return None

    # high level function call
    def set_main_board_trig(self, led_feb_id):
        return self.set_trig_source(led_feb_id, SELECT_MAIN_BOARD)

    # high level function call
    def set_ext_crystal_trig(self, led_feb_id):
        return self.set_trig_source(led_feb_id, SELECT_EXT_CRYSTAL)

    # high level function call
    def set_u_controller_trig(self, led_feb_id):
        return self.set_trig_source(led_feb_id, SELECT_uCONTROLLER)

    # low level function call
    def get_firmware_id(self, led_feb_id):
        if (led_feb_id < self.get_n_led_febs()):
            return self.get_input(self.get_led_feb_addr(led_feb_id), FIRMWARE_VERSION)
        else:
            return None

    # high level function call
    def fetch_firmware_id(self, led_feb_id):
        value = self.get_firmware_id(led_feb_id)
        if (value == None):
            return "Invalid LED ID request or firmware ID"
        else:
            return f"Current firmware version is {value >> 8}.{(value & 0xF0) >> 4}.{value & 0xF}" 

    def __del__(self):
        self.modbus_client.close()
        del self.modbus_client
        if (self.run_mode == True):
            del self.run_ctrl
        
