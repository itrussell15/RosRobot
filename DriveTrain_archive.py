# -*- coding: utf-8 -*-

import rclpy, socket
from rclpy.node import Node

from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from std_srvs.srv import Empty

from raspbot.misc.I2C_Control import LowLevelI2C
from raspbot.misc.enums import *

class DriveTrain(Node):

    def __init__(self, max_val = 150):
        super().__init__("drive_train")
        self._device = LowLevelI2C(0x16, 1)
        self._left = self.DriveSide(max_val)
        self._right = self.DriveSide(max_val)
        self._max_val = max_val

        self.status = DriveStatus.STARTING
        self.state = DriveState.OPERATIONAL

        self._dist_thresh = 0.15

        my_ip = socket.gethostbyname(socket.gethostname())
        self.is_raspi = my_ip == "192.168.0.62"
        if not self.is_raspi:
            self._device = LowLevelI2C(0x16, 1)
        else:
            self._device = FakeLLC()
            sour
        self.speed = 0
        self.spin = 0

        self._CreateConnections()
        self.status = DriveStatus.STOPPED

    # Forward = value > 0
    # Backward = value < 0
    def drive(self, value):
        value = self.clamp(value)
        print("Drive Command Sent: {}%".format(value))
        array = self._left.drive(value)
        array.extend(self._right.drive(value))
        self._device.write_array(0x01, array)
        self.speed = value

    # Left = value > 0
    # Right = value < 0
    def turn(self, value):
        array = self._left.drive(value)
        array.extend(self._right.drive(-value))
        self._device.write_array(int(Registers.MOVE), array)

    def stop(self, stop = True):
        self.speed = 0
        self._set_status(DriveStatus.STOPPED)
        self._device.writeByte(int(Registers.STOP), 0x00)

    def _CreateConnections(self):
        self._cmd_vel_sub = self.create_subscription(
            Twist,
            "cmd_vel",
            self._vel_callback,
            10
        )
        self._range_sub = self.create_subscription(
            Range,
            "range_sensor",
            self._range_sense_callback,
            10
        )
        # self._buzzer_client = self.create_client(
        #     Empty,
        #     "alarm_sound_srv"
        # )
        # while not self._buzzer_client.wait_for_service(timeout_sec = 3.0):
        #     print("unable to find service")

    def _vel_callback(self, msg):
        # self._manage_speed(msg.linear.x)
        self.drive(msg.linear.x)
        self.turn(msg.angular.z)

    # def _range_sense_callback(self, msg):
    #     distance = msg.range
    #     # Pass to RangeManager?
    #     if distance < self._dist_thresh:
    #         self.stop()
    #     # else:
    #     #     self._manage_speed(50)

    # def _send_buzzer_req(self):
    #     req = Empty.Request()
    #     self.future = self._buzzer_client.call_async(req)

    #Sets state of drive system and sends commands to
    # def _manage_speed(self, incoming):
    #     # If the robot is able to be controlled
    #     if self.state == DriveState.OPERATIONAL:
    #         # If the robot is moving and the new command is not 0
    #         if self.status == DriveStatus.IN_MOTION and incoming != 0:
    #             # If the new speed is different than the new speed
    #             if self.speed != incoming:
    #                 print("New Speed Set: {}%".format(incoming))
    #                 self.drive(incoming)
    #                 # self._set_status(DriveStatus.IN_MOTION)
    #         # If the robot is stopped
    #         elif self.status == DriveStatus.STOPPED:

    #             self.drive(incoming)
    #             self._set_status(DriveStatus.IN_MOTION)
    #         else:
    #             print("Already Stopped")
    #             if self.status != DriveStatus.STOPPED:
    #                 print("Stopping")
    #                 self.stop()
    #                 self._set_status(DriveStatus.STOPPED)
    #     else:
    #         pass

    # def _set_status(self, status):
    #     # TODO have a method to check if value exists in enum
    #     # if status in DriveStatus._value2member_map_
    #     self.status = status
    #     # else:
    #     #     raise Errors.UnknownStatusError(status)

    def clamp(self, value):
        # print("Value {} Exceeds Range. Value Clamped.".format(value))
        return int(max(-self._max_val, min(value, self._max_val)))

    class RangeManager:

        def __init__(self, threshs):
            self.min = threshs["min"]
            self.caution = threshs["caution"]

        # Incoming == incoming distance val from sensor
        # returns speed value modifer from 1-100.
        #TODO implement PID?
        def speedModifier(self, incoming):
            if incoming <= self.min:
                return 0
            else:
                if incoming <= self.caution:
                    span = incoming - self.min

    class DriveSide:

        def __init__(self, max_value = 150):
            self._max_value = max_value

        def drive(self, value):
            dir = 1
            if value > 0:
                dir = 0
            return [dir, int(abs(self._drive_value(value)))]

        # Turns 0-100 value into a value that the drivers can use.
        def _drive_value(self, val):
            return self._map_values(val, 0, 100, 0, self._max_value)

        def _RPM_estimate(self, val):
            return self._map_values(val, 0, 100, 0, self._max_rpm)

        @staticmethod
        def _map_values(value, fromMin, fromMax, toMin, toMax):
            # Figure out how 'wide' each range is
            fromSpan = fromMax - fromMin
            toSpan = toMax - toMin

            # Convert the left range into a 0-1 range (float)
            valueScaled = float(value - fromMin) / float(fromSpan)

            # Convert the 0-1 range into a value in the right range.
            return toMin + (valueScaled * toSpan)



def main(args=None):
    #  initialize the wheelie node
    rclpy.init(args=args)
    Drive = DriveTrain()

    #  wait for incoming commands
    rclpy.spin(Drive)

    #  Interrupt detected, shut down
    GPIO.cleanup()
    Drive.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
