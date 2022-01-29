#!/usr/bin/env python
# coding: utf-8

# import YB_Pcb_Car  #Import Yahboom car library
# from DriveTrain import DriveTrain
# from Drive.Servos import Servos
# import time

from misc.I2C_Control import LowLevelI2C
from misc.enums import Registers
import time

llc = LowLevelI2C(0x16, 1)

def drive(bus, value):
    print("Drive Command Sent: {}%".format(value))
    # array = self._left.drive(value)
    # array.extend(self._right.drive(value))
    array = [0, value, 0, value]
    bus.write_array(0x01, array)

def stop():
    bus.writeByte(int(Registers.STOP), 0x00)

drive(llc, 50)
time.sleep(2)
stop()
