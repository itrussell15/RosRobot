import smbus
from .Errors import *


class LowLevelI2C:

    def __init__(self, address, bus):
        self._device = self._get_i2c_device(0x16, 1)

    def _get_i2c_device(self, address, bus):
        self._addr = address
        if bus:
            return smbus.SMBus(bus)
        else:
            return smbus.SMBus(1)

    def writeByte(self, reg, data):
        try:
            self._device.write_byte_data(self._addr, reg, data)
        except Exception as e:
            raise I2C_Error(e)
            # pass

    def write_array(self, reg, data):
        try:
            self._device.write_i2c_block_data(self._addr, reg, data)
        except Exception as e:
            raise I2C_Array_Error(e)
            # pass
            
class FakeLLC:

    def __init__(self):
        print("Fake Device Ready!")
    
    def write_array(self, reg, data):
        print("Array Data: {}".format(data))
        
    def writeByte(self, reg, data):
        print("Byte Data: {}".format(data))