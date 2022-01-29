#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Jan 27 09:23:27 2022

@author: schmuck
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range

import socket
from enum import Enum

from misc.I2C_Control import LowLevelI2C
from misc.enums import *

class DriveController(Node):

    def __init__(self, max_val = 150):
        super().__init__("drive_train")

        thresholds = {"slow": 0.15, "stop": 0.10}
        self.manager = self.DriveManager(thresholds)
        self.driveTrain = self.DriveTrain()
        self._driveOverride = False

        self._CreateLinks()

        self._speed = 0.0
        self._spin = 0.0
        
        self.status = self.DriveStatus.STOPPED
        self.stop_timeout = 3
        self.stopped_count = 0
        self.stopped_timer = None

    def _vel_callback(self, msg):
        self._speed = self.manager.get_speed(msg.linear.x)
        self._turn = msg.angular.z
        
        # if spin != 0 or self._speed != 0:
        #     # Checks for biggest value to perform a move
        #     if abs(self._speed) > 0:
        #         print("Forward: {}".format(self._speed))
        #         self.driveTrain.drive(self._speed)                
        #     else:
        #         print("Turning: {}".format(self._turn))
        #         self.driveTrain.turn(self._turn)
        # else:
        #     print("Stopping")
        #     self.driveTrain.stop()
        
    def _sensor_speed_change(self, msg):
        self.manager.range_callback(msg)
        new_speed = self.manager.get_speed(self._speed)
        if not self._driveOverride:
            if new_speed != self._speed:
                self._speed = new_speed
        
        self._drive()
        
    def _drive(self):
        if self._spin != 0 or self._speed != 0:
            # Checks for biggest value to perform a move
            if abs(self._speed) > 0:
                print("Forward: {}".format(self._speed))
                self.driveTrain.drive(self._speed)                
            else:
                print("Turning: {}".format(self._turn))
                self.driveTrain.turn(self._turn)
            self.status = self.DriveStatus.IN_MOTION
            if self.stopped_timer in self.timers:
                self.destroy_timer(self.stoppped_timer)
        else:
            print("Stopping")
            self.driveTrain.stop()
            self.status = self.DriveStatus.STOPPED
            if not self.manager.directions["forward"] and self.stopped_timer not in self.timers:
                print("Creating blocked timer")
                self.stopped_timer = self.create_timer(1, self._stopped_handler)

    def _stopped_handler(self):
        if self.stopped_count <= self.stop_timeout:
            self.stopped_count += 1
        else:
            print("blocked time exceeded")
            self.end_stop = self.create_timer(2, self._continue)
            self.stopped_count = 0
            self._driveOverride = True
            self._speed = -0.25
            self.stopped_timer.destroy()
            self.stopped_timer = None

    def _continue(self):
        print("HERE")
        self.stop()
        self.destroy_timer(self.end_stop)
        self._driveOverride = False
        

    def stop(self):
        self.driveTrain.stop()
        self._speed = 0

    def _CreateLinks(self):
        self._cmd_vel_sub = self.create_subscription(
            Twist,
            "cmd_vel",
            self._vel_callback,
            10
        )
        self._range_sensor_sub = self.create_subscription(
            Range,
            "range_sensor",
            self._sensor_speed_change,
            10
        )

    # def drive(self, value):
    #     print("Drive Command Sent: {}%".format(value))
    #     array = self._left.drive(value)
    #     array.extend(self._right.drive(value))
    #     self._device.write_array(int(Registers.MOVE), array)
    #     self.speed = value

    # def turn(self, value):
    #     array = self._left.drive(value)
    #     array.extend(self._right.drive(-value))
    #     self._device.write_array(int(Registers.MOVE), array)

    # def stop(self, stop = True):
    #     self.speed = 0
    #     # self._set_status(DriveStatus.STOPPED)
    #     self._device.writeByte(int(Registers.STOP), 0x00)

    class DriveStatus(Enum):
        IN_MOTION = 0
        STOPPED = 1    

    class DriveTrain:
        
        def __init__(self):
            self._llc = LowLevelI2C(0x16, 1)
            self._right = self.DriveSide()
            self._left = self.DriveSide()
            
        def drive(self, value):
            # print("Drive Command Sent: {}%".format(value))
            array = self._left.drive(value)
            array.extend(self._right.drive(value))
            self._llc.write_array(int(Registers.MOVE), array)
            
        def turn(self, value):
            array = self._left.drive(value)
            array.extend(self._right.drive(-value))
            self._llc.write_array(int(Registers.MOVE), array)
        
        def stop(self, stop = True):
            self._llc.writeByte(int(Registers.STOP), 0x00)
            
    
            
        class DriveSide:
    
                def __init__(self, max_value = 150):
                    self._max_value = max_value
    
                def drive(self, value):
                    dir = 1
                    if value > 0:
                        dir = 0
                    return [dir, int(abs(self._drive_value(value)))]
    
                # Turns 0-1 value into a value that the drivers can use.
                def _drive_value(self, val):
                    return self._map_values(val, 0, 1.0, 0, self._max_value)
    
                @staticmethod
                def _map_values(value, fromMin, fromMax, toMin, toMax):
                    # Figure out how 'wide' each range is
                    fromSpan = fromMax - fromMin
                    toSpan = toMax - toMin
    
                    # Convert the left range into a 0-1 range (float)
                    valueScaled = float(value - fromMin) / float(fromSpan)
    
                    # Convert the 0-1 range into a value in the right range.
                    return toMin + (valueScaled * toSpan)

    class DriveManager:

        def __init__(self, thresh):
            self.stop_dist = thresh["stop"]
            self.slow_dist = thresh["slow"]
            self._dist = 1.0
            
            self._speed_mod = 0.0
            
            self._speed = 0.0
            self.turn = 0.0
            
            self.directions = {
                "forward": True,
                "backward": True,
                "left": True,
                "right": True
                }
            
        # def stopped_handler(self):
        #     if self._stop_count <= 5:  
        #         if not self.directions["forward"]:
        #             print('Robot Stopped for {} seconds'.format(self.stop_count))
        #             self._stop_count += 1
        #         else:
        #             self._stop_count = 0
        #     else:
        #         print('backup')
                

        def range_callback(self, msg):
            dist = msg.range
            
            self._speed_mod = 1.0
            # If it is in the stop zone --> Stop forward motion
            if self._close_enough(dist, 0.01):
                self.directions["forward"] = False
                
            # allow forward motion
            else:
                # allow forward motion
                self.directions["forward"] = True
                # if in speed restriction zone
                if dist <= self.slow_dist:
                    # change speed_modifier
                    self._speed_mod = self._speed_modifier(dist) 
                    
                         
        def _speed_modifier(self, dist):
            return (dist - self.stop_dist) / \
                (self.slow_dist - self.stop_dist)
        
        def get_speed(self, speed):
            self._speed = speed
            # Only makes changes to positive motion
            if speed > 0:
                # changes input speed by the speed modifier
                self._speed = self._speed_mod * self._speed
            # if forward motion is not allowed and forward motion attempted
                if not self.directions["forward"]:
                    # set speed to 0
                    self._speed = 0
                
            return self._speed
        
        def _close_enough(self, val, slop):
            return val < self.stop_dist + slop



def main(args=None):


    #  initialize the wheelie node
    rclpy.init(args=args)

    Drive = DriveController()
    # Drive.drive(50)
    # time.sleep(2)
    # Drive.sto

    #  wait for incoming commands
    print("Spinning")
    rclpy.spin(Drive)

    #  Interrupt detected, shut down
    GPIO.cleanup()
    Drive.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
