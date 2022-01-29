# -*- coding: utf-8 -*-

class I2C_Error(Exception):

    def __init__(self, incoming):
        message = "ERROR WRITING TO I2C\n{}".format(incoming)
        super().__init__(message)

class I2C_Array_Error(Exception):

    def __init__(self, incoming):
        message = "ERROR WRITING TO I2C ARRAY\n{}".format(incoming)
        super().__init__(message)

class Servo_Control_Error(Exception):

    def __init__(self, incoming):
        message = "I2C Error Occured While Trying to Control the Servo\n{}".format(incoming)
        super().__init__(message)

class UnknownStatusError(Exception):

    def __init__(self, incoming):
        message = "{} is not a valid member DriveStatus"
        super().__init__(message)
