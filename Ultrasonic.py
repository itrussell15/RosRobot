import RPi.GPIO as GPIO
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import String
from sensor_msgs.msg import Range

import time, math
from PID import PI_Control

class SensorRead(Node):

    def __init__(self, echo = 24, trigger = 23, freq = 10):
        super().__init__("sensor_read")
        self.range_sensor, self._range_msg = self._configureUltrasonic(echo, trigger, freq)
        self.PID = PI_Control(kp = 0.9, ki = 0.1)
        self.PID.set_target(0.75)
        self._ultraSensor = self.UltrasonicSensor()
        self.start()

    def _configureUltrasonic(self, hz, echo, trigger):
        self._hz = hz
        self._distance_sensor = self.UltrasonicSensor(
            echo = echo,
            trigger = trigger)
        distance_publisher = self.create_publisher(Range, "range_sensor", 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, "cmd_vel", 10)
        msg = Range()
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = self._distance_sensor.angle
        msg.min_range = self._distance_sensor.min_range
        msg.max_range = self._distance_sensor.max_range
        return distance_publisher, msg

    def _range_callback(self):
        self._range_msg.range = self._ultraSensor.read()
        self.range_sensor.publish(self._range_msg)
        
        twist_msg = Twist()
        twist_msg.linear.x = self._map_values(self.PID.gain(self._range_msg), -1000, 1000, 0, 1)
        self.cmd_vel_publisher.publish(twist_msg)
        
    @staticmethod
    def _map_values(value, fromMin, fromMax, toMin, toMax):
        # Figure out how 'wide' each range is
        fromSpan = fromMax - fromMin
        toSpan = toMax - toMin

        # Convert the left range into a 0-1 range (float)
        valueScaled = float(value - fromMin) / float(fromSpan)

        # Convert the 0-1 range into a value in the right range.
        return toMin + (valueScaled * toSpan)

    def start(self):
        self._distance_timer = self.create_timer(1.0 / self._hz, self._range_callback)

    def stop(self):
        self.destroy_timer(self._distance_timer)

    class UltrasonicSensor:

        def __init__(self, echo = 24, trigger = 23):
            self._echo = echo
            self._trigger = trigger
            self._speedSound = 343.0
            self.min_range = 0.030
            self.max_range = 4.0000
            self.angle = 15.0 * math.pi / 180.0

            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)
            GPIO.setup(trigger, GPIO.OUT)
            GPIO.setup(echo, GPIO.IN)

        def read(self):

            GPIO.output(self._trigger, True)
            time.sleep(0.00001)
            GPIO.output(self._trigger, False)

            start = time.time()
            while GPIO.input(self._echo) == 0:
                start = time.time()
            stop = time.time()
            while GPIO.input(self._echo) == 1:
                stop = time.time()
                if stop - start >= 0.04:
                    stop = start
                    break
            out = stop - start
            return out * self._speedSound / 2

def main(args = None):
    rclpy.init(args = args)
    s = SensorRead()

    print("Spinning")
    rclpy.spin(s)

    GPIO.cleanup()
    s.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
