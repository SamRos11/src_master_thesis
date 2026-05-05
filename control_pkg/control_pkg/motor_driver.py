#!/usr/bin/env python3

import os
# Essential configuration for Jetson pins
os.environ['BLINKA_JETSON_BOARD'] = 'JETSON_ORIN_NANO'

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from adafruit_servokit import ServoKit
import busio
import board
import adafruit_pca9685

# Hardware calibrated constants
ESC_BACKWARD_MAX_US = 1250  # µs — max backward
ESC_IDLE_US         = 1400  # µs — idle / neutral
ESC_FORWARD_MAX_US  = 1450  # µs — max forward
ESC_CHANNEL         = 8
SERVO_CHANNEL       = 4
SERVO_CENTER_ANGLE  = 50.0  # Center angle (neutral) for steering

class MotorDriverNode(Node):
    def __init__(self):
        super().__init__('motor_driver_node')
        
        # --- NAV2 PARAMETERS ---
        # Nav2 uses meters/second. We need to know the maximum speed
        # to map it to the -1.0 / 1.0 range required by the ESC
        self.declare_parameter('max_linear_speed', 1.0)  # meters per second
        self.declare_parameter('max_angular_speed', 1.0) # radians per second
        
        # --- HARDWARE SETUP ---
        self.get_logger().info("Initializing I2C and PCA9685...")
        i2c = busio.I2C(board.SCL, board.SDA)
        self.pca = adafruit_pca9685.PCA9685(i2c)
        self.pca.frequency = 50
        
        self.kit = ServoKit(channels=16)
        self.kit.servo[SERVO_CHANNEL].actuation_range = 180
        self.kit.servo[SERVO_CHANNEL].set_pulse_width_range(400, 2000)
        
        # Initial arming (Stop and wheels straight)
        self._set_esc(0.0)
        self.kit.servo[SERVO_CHANNEL].angle = SERVO_CENTER_ANGLE
        
        # --- NAV2 CONNECTION ---
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # --- SAFETY SYSTEM (Watchdog) ---
        # If Nav2 crashes, the car MUST stop.
        # This timer stops the motors if no messages are received for 0.5 seconds.
        self.last_cmd_time = self.get_clock().now()
        self.watchdog_timer = self.create_timer(0.5, self.watchdog_callback)
        
        self.get_logger().info("Hardware Armed! Listening on /cmd_vel...")

    def _set_esc(self, speed: float) -> None:
        """ Maps percentage speed to PWM pulses for the ESC """
        # Ensure speed is between -1.0 and 1.0
        speed = max(-1.0, min(1.0, speed))
        
        if speed < 0:
            us = ESC_IDLE_US + speed * (ESC_IDLE_US - ESC_BACKWARD_MAX_US)
        else:
            us = ESC_IDLE_US + speed * (ESC_FORWARD_MAX_US - ESC_IDLE_US)
            
        us = max(ESC_BACKWARD_MAX_US, min(ESC_FORWARD_MAX_US, int(us)))
        period_us = 1_000_000 / self.pca.frequency
        self.pca.channels[ESC_CHANNEL].duty_cycle = int(us / period_us * 65535)

    def cmd_vel_callback(self, msg: Twist):
        """ Receives commands from Nav2 and translates them for the motors """
        # Log the received command to verify communication
        self.get_logger().info(f"Received command: Linear {msg.linear.x}, Angular {msg.angular.z}")
        
        # Update safety timer
        self.last_cmd_time = self.get_clock().now()
        
        max_lin = self.get_parameter('max_linear_speed').value
        max_ang = self.get_parameter('max_angular_speed').value
        
        # 1. THROTTLE TRANSLATION (Proportional and Smooth!)
        # E.g. if Nav2 asks for 0.5 m/s and max is 1.0 m/s -> speed_percent = 0.5 (50%)
        speed_percent = msg.linear.x / max_lin
        self._set_esc(speed_percent)
        
        # 2. STEERING TRANSLATION
        # Convert angular velocity to a percentage [-1.0, 1.0]
        steer_percent = msg.angular.z / max_ang
        steer_percent = max(-1.0, min(1.0, steer_percent))
        
        # The safe range is from 0 to 100 with 50 at the center.
        # Multiply percentage by 50 to move from the center.
        angle = SERVO_CENTER_ANGLE + (steer_percent * 50)
        
        # Mechanically limit between 0 and 100
        angle = max(0.0, min(100.0, angle))
        self.kit.servo[SERVO_CHANNEL].angle = angle

    def watchdog_callback(self):
        """ Stops the car in case of Nav2 disconnection or crash """
        time_since_last_cmd = (self.get_clock().now() - self.last_cmd_time).nanoseconds
        if time_since_last_cmd > 500_000_000: # 0.5 seconds in nanoseconds
            self._set_esc(0.0)
            self.kit.servo[SERVO_CHANNEL].angle = SERVO_CENTER_ANGLE

def main(args=None):
    rclpy.init(args=args)
    node = MotorDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Physically stop the motors when closing the program with Ctrl+C
        node._set_esc(0.0)
        node.kit.servo[SERVO_CHANNEL].angle = SERVO_CENTER_ANGLE
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()