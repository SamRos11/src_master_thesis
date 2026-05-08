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
ESC_BACKWARD_MAX_US = 1250  # µs - max backward
ESC_IDLE_US         = 1400  # µs - idle / neutral
ESC_FORWARD_MAX_US  = 1450  # µs - max forward
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
        
        # Fix: Share the same I2C bus between PCA9685 and ServoKit to avoid I2C access crashes
        self.pca = adafruit_pca9685.PCA9685(i2c)
        self.pca.frequency = 50
        
        self.kit = ServoKit(channels=16, i2c=i2c)
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
        # Update safety timer
        self.last_cmd_time = self.get_clock().now()
        
        # --- HARDWARE SAFETY CLAMP ---
        # Even if Nav2 sends 2.0 m/s, we physically clamp it here to prevent runaway
        safe_linear_x = max(-0.3, min(msg.linear.x, 0.3))
        safe_angular_z = max(-0.8, min(msg.angular.z, 0.8))
        
        #self.get_logger().info(f"Received: Lin {msg.linear.x:.2f}, Ang {msg.angular.z:.2f} | Clamped: Lin {safe_linear_x:.2f}, Ang {safe_angular_z:.2f}")

        max_lin = self.get_parameter('max_linear_speed').value
        max_ang = self.get_parameter('max_angular_speed').value
        
        # 1. THROTTLE TRANSLATION (Proportional and Smooth)
        # Use the clamped value to calculate the percentage
        speed_percent = safe_linear_x / max_lin
        self._set_esc(speed_percent)
        
        # 2. STEERING TRANSLATION
        # Convert clamped angular velocity to a percentage [-1.0, 1.0]
        steer_percent = safe_angular_z / max_ang
        steer_percent = max(-1.0, min(1.0, steer_percent))
        
        # The safe range is from 0 to 100 with 50 at the center.
        # Multiply percentage by 50.0 to move from the center.
        angle = SERVO_CENTER_ANGLE + (steer_percent * 50.0)
        
        # Mechanically limit between 0.0 and 100.0
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