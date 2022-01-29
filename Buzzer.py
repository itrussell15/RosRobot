#ROS 2
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import Empty

import RPi.GPIO as GPIO
import time

class BuzzerNode(Node):

    def __init__(self, pin = 32):
        super().__init__("alarm_sound")
        self._buzzer = self.Buzzer(pin)
        self.service = self.create_service(Empty,
        "alarm_sound_srv",
        self.quick_buzz)

    def quick_buzz(self, req, resp):
        print("HERE!!")
        if not self._buzzer.on:
            self._buzzer.start()
            time.sleep(0.5)
            self._buzzer.stop()

        return Empty.Response()

    def _alarm_callback(self, msg):
        if (msg.data):
            self._buzzer.start()
        else:
            self._buzzer.stop()

    class Buzzer:

        def __init__(self, pin):
            self.pin = pin
            GPIO.setwarnings(False)
            GPIO.setmode(GPIO.BOARD)
            GPIO.setup(pin, GPIO.OUT)
            self._p = GPIO.PWM(self.pin, 1000)
            self.on = False

        def start(self):
            self._p.start(50)
            self.on = True

        def stop(self):
            self._p.stop()
            self.on = False

def main(args = None):
    rclpy.init(args = args)
    buzzNode = BuzzerNode()

    rclpy.spin(buzzNode)
    GPIO.cleanup()
    buzzNode.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

# b = Buzzer()
#
# b.start()
# time.sleep(5)
# b.stop()
