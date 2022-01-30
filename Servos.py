# -*- coding: utf-8 -*-

import I2C_Control
import enums

class ServoControl(Node):

    def __init__(self):
        super().__init__("servo-control")

    class Servos:

        def __init__(self):
            self._device =  I2C_Control.LowLevelI2C(0x16, 1)
            self.move_top(45)
            self.move_bottom(45)

        def move_top(self, angle):
            self._device.write_array(
                enums.Registers.SERVO, [int(enums.Servos.TOP), self.clamp(angle)]
            )
            print("TOP MOVED TO {}".format(angle))

        def move_bottom(self, angle):
            self._device.write_array(
                enums.Registers.SERVO, [int(enums.Servos.BOTTOM), self.clamp(angle)]
            )
            print("BOTTOM MOVED TO {}".format(angle))

        @staticmethod
        def clamp(value, min = 0, max = 180):
            return int(max(min, min(value, max)))
