"""Alternative direct Pi GPIO motor output; Arduino must have drive_motors=false."""
import time

from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from .drive import motor_duties, wheel_speeds
from .runtime import param, run


class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')
        self.mock = param(self, 'mock', True)
        self.track = param(self, 'track_width', 0.55)
        self.max_speed = param(self, 'max_speed', 1.0)
        self.timeout = param(self, 'command_timeout', 0.5)
        self.pwm_pins = param(self, 'pwm_pins', [9, 10, 6, 5])
        self.dir_pins = param(self, 'direction_pins', [8, 11, 7, 4])
        if len(self.pwm_pins) != 4 or len(self.dir_pins) != 4 or min(self.track, self.max_speed, self.timeout) <= 0:
            raise ValueError('expected four PWM/direction pins and positive drive parameters')
        self.gpio = None
        self.pwms = []
        if not self.mock:
            import RPi.GPIO as GPIO
            self.gpio = GPIO
            GPIO.setmode(GPIO.BCM)
            for pwm_pin, dir_pin in zip(self.pwm_pins, self.dir_pins):
                GPIO.setup(pwm_pin, GPIO.OUT)
                GPIO.setup(dir_pin, GPIO.OUT)
                pwm = GPIO.PWM(pwm_pin, 100)
                pwm.start(0)
                self.pwms.append(pwm)
        self.linear = self.angular = 0.0
        self.last_command = float('-inf')
        self.pub = self.create_publisher(Float64MultiArray, 'gpio_motor_duty_cycle', 10)
        self.create_subscription(Twist, 'cmd_vel', self.command, 10)
        self.create_timer(0.05, self.update)

    def command(self, msg):
        self.linear, self.angular = msg.linear.x, msg.angular.z
        self.last_command = time.monotonic()

    def update(self):
        v, w = self.linear, self.angular
        if time.monotonic() - self.last_command > self.timeout:
            v = w = 0.0
        duties = motor_duties(*wheel_speeds(v, w, self.track, self.max_speed), self.max_speed)
        self.pub.publish(Float64MultiArray(data=duties))
        if self.gpio:
            for pwm, pin, duty in zip(self.pwms, self.dir_pins, duties):
                self.gpio.output(pin, duty >= 0)
                pwm.ChangeDutyCycle(abs(duty))

    def destroy_node(self):
        for pwm in self.pwms:
            pwm.ChangeDutyCycle(0)
            pwm.stop()
        if self.gpio:
            self.gpio.cleanup(self.pwm_pins + self.dir_pins)
        return super().destroy_node()


def main(args=None):
    run(MotorController, args)
