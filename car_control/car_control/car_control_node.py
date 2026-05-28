#!/usr/bin/env python3

import argparse
import logging
import math

import board
import busio
from adafruit_servokit import ServoKit
import colorlog
import rclpy
from geometry_msgs.msg import Twist
from rclpy.logging import LoggingSeverity
from rclpy.node import Node
from std_msgs.msg import Float64, Int64


class CarControlNode(Node):
    DEFAULT_QOS_DEPTH = 10
    CMD_VEL_TOPIC = '/cmd_vel'
    # If no Twist on /cmd_vel for this long, hold ref_speed at 0 (repeated while stale)
    # and centre steering once on entering stale state (does not re-command servo every tick).
    CMD_VEL_TIMEOUT_SEC = 1
    CMD_VEL_WATCHDOG_PERIOD_SEC = 0.05
    MICROROS_PING_PERIOD_SEC = 5.0
    MICROROS_PING_TELEMETRY_MAX_AGE_SEC = 2.0

    # Servo — controlled via ServoKit (angle in degrees)
    SERVO_CHANNEL = 4
    SERVO_ACTUATION_RANGE = 180  # degrees — full physical sweep of the servo
    SERVO_MIN_DEG = 0  # degrees — full left
    SERVO_CENTER_DEG = 50  # degrees — straight ahead
    SERVO_MAX_DEG = 100  # degrees — full right
    SERVO_MIN_US = 400  # µs — pulse width at 0°
    SERVO_MAX_US = 2000  # µs — pulse width at 180°

    STEERING_MIN = -math.pi / 2  # max left  (~-1.5708 rad)
    STEERING_MAX = math.pi / 2  # max right (~+1.5708 rad)

    def __init__(self, verbose: bool = True):
        super().__init__('car_control_node')

        # Suppress the duplicate rcutils console output for this node;
        # all logging goes through colorlog (self.logger) instead.
        self.get_logger().set_level(LoggingSeverity.FATAL)
        self._setup_logger(verbose)

        self.ref_speed_publisher = self.create_publisher(
            Float64,
            '/ref_speed',
            self.DEFAULT_QOS_DEPTH,
        )

        self.create_subscription(
            Twist,
            self.CMD_VEL_TOPIC,
            self._cmd_vel_callback,
            self.DEFAULT_QOS_DEPTH,
        )
        self.measured_speed_subscription = self.create_subscription(
            Float64,
            '/measured_speed',
            self._measured_speed_callback,
            self.DEFAULT_QOS_DEPTH,
        )
        self.encoder_count_subscription = self.create_subscription(
            Int64,
            '/encoder_count',
            self._encoder_count_callback,
            self.DEFAULT_QOS_DEPTH,
        )
        self.control_force_subscription = self.create_subscription(
            Float64,
            '/control_force',
            self._control_force_callback,
            self.DEFAULT_QOS_DEPTH,
        )

        self.latest_ref_speed_mps = 0.0
        self.latest_measured_speed_mps = None
        self.latest_encoder_count = None
        self.latest_control_force = None

        self._last_cmd_vel_time = self.get_clock().now()
        self._cmd_vel_idle_active = False
        self._last_mcu_telemetry_time = None

        self._init_i2c()

        self._cmd_vel_watchdog = self.create_timer(
            self.CMD_VEL_WATCHDOG_PERIOD_SEC,
            self._cmd_vel_watchdog_callback,
        )
        self._microros_ping_timer = self.create_timer(
            self.MICROROS_PING_PERIOD_SEC,
            self._microros_ping_callback,
        )

        self.logger.info(
            'car_control_node started — publishing /ref_speed (Float64) and '
            f'listening to {self.CMD_VEL_TOPIC} (Twist), '
            '/measured_speed, /encoder_count, /control_force'
        )

    # ------------------------------------------------------------------
    # Logger setup
    # ------------------------------------------------------------------

    def _setup_logger(self, verbose: bool = True):
        """Configure a colorlog handler with human-readable timestamps."""
        self.logger = logging.getLogger(self.get_name())
        self.logger.setLevel(logging.DEBUG)
        self.logger.handlers.clear()

        if not verbose:
            self.logger.addHandler(logging.NullHandler())
            return

        handler = logging.StreamHandler()
        handler.setFormatter(colorlog.ColoredFormatter(
            fmt='%(log_color)s%(levelname)-8s%(reset)s '
                '%(blue)s[%(asctime)s]%(reset)s '
                '%(cyan)s[%(name)s]%(reset)s: %(message)s',
            datefmt='%Y-%m-%d %H:%M:%S',
            log_colors={
                'DEBUG': 'cyan',
                'INFO': 'green',
                'WARNING': 'yellow',
                'ERROR': 'red',
                'CRITICAL': 'red,bg_white',
            },
        ))
        self.logger.addHandler(handler)

    # ------------------------------------------------------------------
    # I2C initialisation
    # ------------------------------------------------------------------

    def _init_i2c(self):
        """Open the I2C bus and configure ServoKit for the servo."""
        self.logger.info(f'I2C bus — SCL: {board.SCL}  SDA: {board.SDA}')
        self._i2c = busio.I2C(board.SCL, board.SDA)

        # ServoKit: handles PWM internals for the steering servo
        self.myKit = ServoKit(channels=16)
        self.myKit.servo[self.SERVO_CHANNEL].actuation_range = self.SERVO_ACTUATION_RANGE
        self.myKit.servo[self.SERVO_CHANNEL].set_pulse_width_range(
            self.SERVO_MIN_US, self.SERVO_MAX_US
        )

        # Centre servo before any command arrives
        self.myKit.servo[self.SERVO_CHANNEL].angle = self.SERVO_CENTER_DEG
        self.logger.info('I2C initialised — servo centred at 50°')

    # ------------------------------------------------------------------
    # ROS callbacks
    # ------------------------------------------------------------------

    def _cmd_vel_callback(self, msg: Twist):
        """Publish reference speed (m/s) to /ref_speed and steer via servo (angular.z rad)."""
        ref_speed_mps = float(msg.linear.x)
        steering_angle = msg.angular.z

        if not math.isfinite(ref_speed_mps):
            self.logger.warning('Ignoring non-finite ref_speed from /cmd_vel')
            return

        self._last_cmd_vel_time = self.get_clock().now()
        self._cmd_vel_idle_active = False

        # Clamp steering angle
        steering_angle = max(self.STEERING_MIN, min(self.STEERING_MAX, steering_angle))

        self.latest_ref_speed_mps = ref_speed_mps
        self.ref_speed_publisher.publish(Float64(data=ref_speed_mps))
        self._set_servo_angle(steering_angle)

    def _cmd_vel_watchdog_callback(self):
        """Hold ref_speed at 0 when /cmd_vel goes stale; centre steering only once."""
        elapsed_s = (
            self.get_clock().now() - self._last_cmd_vel_time
        ).nanoseconds * 1e-9
        if elapsed_s <= self.CMD_VEL_TIMEOUT_SEC:
            self._cmd_vel_idle_active = False
            return

        # Quando il robot va in timeout, azzera la memoria della velocità
        self.latest_ref_speed_mps = 0.0
        self._last_published_speed = 0.0 
        self.ref_speed_publisher.publish(Float64(data=0.0))

        if not self._cmd_vel_idle_active:
            self.logger.warning(
                'No recent /cmd_vel — holding ref_speed=0 m/s; steering centred once'
            )
            self._set_servo_angle(0.0)
            self._cmd_vel_idle_active = True

    def _touch_mcu_telemetry(self):
        self._last_mcu_telemetry_time = self.get_clock().now()

    def _microros_ping_callback(self):
        if self._last_mcu_telemetry_time is None:
            self.logger.info('micro-ROS: waiting for MCU…')
            return

        age_s = (
            self.get_clock().now() - self._last_mcu_telemetry_time
        ).nanoseconds * 1e-9
        if age_s <= self.MICROROS_PING_TELEMETRY_MAX_AGE_SEC:
            self.logger.info('micro-ROS: OK')
        else:
            self.logger.warning(
                f'micro-ROS: no recent telemetry ({age_s:.1f}s)'
            )

    def _measured_speed_callback(self, msg: Float64):
        self._touch_mcu_telemetry()
        self.latest_measured_speed_mps = float(msg.data)
        self._log_state('measured_speed')

    def _encoder_count_callback(self, msg: Int64):
        self._touch_mcu_telemetry()
        self.latest_encoder_count = int(msg.data)
        self._log_state('encoder_count')

    def _control_force_callback(self, msg: Float64):
        self._touch_mcu_telemetry()
        self.latest_control_force = float(msg.data)
        self._log_state('control_force')

    # ------------------------------------------------------------------
    # Hardware interface — Servo
    # ------------------------------------------------------------------

    def _set_servo_angle(self, angle: float):
        """
        Send a steering angle to the servo via I2C.

        Parameters
        ----------
        angle : float
            Steering angle in radians, in [-pi/2, pi/2].

        """
        degrees = self._angle_to_degrees(angle)
        #self.logger.debug(f'Servo angle: {angle:+.4f} rad  =>  {degrees:.1f}°')
        self.myKit.servo[self.SERVO_CHANNEL].angle = degrees

    def _angle_to_degrees(self, angle: float) -> float:
        """
        Map steering angle [-pi/2, pi/2] to servo degrees [0, 100].

        -pi/2  →   0°  (full left)
         0     →  50°  (straight ahead)
        +pi/2  → 100°  (full right)

        """
        normalised = angle / (math.pi / 2)  # rescale to [-1, 1]
        degrees = self.SERVO_CENTER_DEG - normalised * self.SERVO_CENTER_DEG
        return max(float(self.SERVO_MIN_DEG), min(float(self.SERVO_MAX_DEG), degrees))

    def _log_state(self, source: str):
        measured_speed = self._format_float(self.latest_measured_speed_mps, suffix=' m/s')
        encoder_count = self._format_int(self.latest_encoder_count)
        control_force = self._format_float(self.latest_control_force)
        ref_speed = self._format_float(self.latest_ref_speed_mps, suffix=' m/s')

        #self.logger.info(
        #    f'{source}: measured_speed={measured_speed} | '
        #    f'encoder_count={encoder_count} | '
        #    f'control_force={control_force} | '
        #    f'ref_speed={ref_speed}'
        #)

    # ------------------------------------------------------------------
    # Utilities
    # ------------------------------------------------------------------

    @staticmethod
    def _format_float(value, suffix: str = '') -> str:
        if value is None:
            return 'n/a'
        return f'{value:+.3f}{suffix}'

    @staticmethod
    def _format_int(value) -> str:
        if value is None:
            return 'n/a'
        return str(value)

    # ------------------------------------------------------------------
    # Shutdown
    # ------------------------------------------------------------------

    def destroy_node(self):
        """Return hardware to a safe state and release I2C resources."""
        if hasattr(self, 'myKit'):
            self.myKit.servo[self.SERVO_CHANNEL].angle = self.SERVO_CENTER_DEG  # centre servo
        self.logger.info('car_control_node shutting down')
        super().destroy_node()


# ----------------------------------------------------------------------
# Entry point
# ----------------------------------------------------------------------

def main(args=None):
    parser = argparse.ArgumentParser(
        description=(
            'car_control_node — ROS 2 low-level bridge for the autonomous car.\n'
            'Subscribes to /cmd_vel (geometry_msgs/msg/Twist), publishes /ref_speed '
            '(std_msgs/msg/Float64) to the micro-ROS client, reads measured_speed / '
            'encoder_count / control_force, and drives the steering servo over I2C.\n'
            'If /cmd_vel is quiet longer than 0.5 s, ref_speed is held at 0; steering '
            'is centred once (servo is not re-written on every watchdog tick).'
        ),
        epilog=(
            '  Topic     : /cmd_vel  (geometry_msgs/msg/Twist)\n'
            '  ' + ('-' * 60) + '\n'
            '  linear.x  : reference wheel speed in m/s (matches client ref_speed)\n'
            '  angular.z : steering angle [-pi/2 ... +pi/2] rad\n'
            '                -1.5708 = full left\n'
            '                 0.0    = straight ahead\n'
            '                +1.5708 = full right\n'
            '  ' + ('-' * 60) + '\n'
            '  All other Twist fields are ignored.\n'
            '\n'
            '  If no messages on /cmd_vel for 0.5 s: ref_speed stays 0; steering centres once.\n'
            '\n'
            '  Quick test:\n'
            '    ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \\\n'
            '      "{linear: {x: 0.5}, angular: {z: 0.785}}"'
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        '-q', '--quiet',
        action='store_true',
        help='Suppress all log output'
    )
    parsed, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args if ros_args else args)
    controller = CarControlNode(verbose=not parsed.quiet)
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()