import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from tracking_msgs.msg import PersonTrackerArray 
from cv_bridge import CvBridge
import numpy as np
from scipy.spatial.transform import Rotation as R
from rclpy.qos import qos_profile_sensor_data
import math

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        
        # Variables
        self.target_id = 1
        self.camera_intrinsics = None
        self.latest_depth_img = None
        self.latest_robot_pose = None
        
        # We only need one memory variable for the threshold logic
        self.last_sent_goal = None
        
        # Thresholds
        self.MIN_MOVE_THRESHOLD = 0.4
        self.SAFE_DISTANCE = 1.5
        self.MAP_BOUNDS = 19.5  # Sanity Check Boundary
        
        self.bridge = CvBridge()
        self.target_was_lost = False

        # Watchdog
        self.TARGET_TIMEOUT_SEC = 2.0
        self.last_target_time = self.get_clock().now()
        self.is_stopped_by_watchdog = False
        
        self.watchdog_timer = self.create_timer(0.5, self.watchdog_callback)

        ## SUBSCRIBERS
        self.create_subscription(CameraInfo, '/camera/camera/color/camera_info', self.info_callback, 10)
        self.create_subscription(PersonTrackerArray, '/tracking/people', self.tracking_callback, 10)
        self.create_subscription(Odometry, '/odometry/global', self.pose_callback, qos_profile_sensor_data)
        self.create_subscription(Image, '/camera/camera/aligned_depth_to_color/image_raw', self.depth_callback, qos_profile_sensor_data)

        ## PUBLISHERS
        self.goal_publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

    def stop_robot(self):
        """ Publishes zero velocity to stop the robot immediately """
        cmd = Twist()
        self.cmd_vel_publisher.publish(cmd)
        self.get_logger().info("Robot stopped.", once=True)

    def info_callback(self, msg):
        if self.camera_intrinsics is None:
            self.camera_intrinsics = np.array(msg.k).reshape((3, 3))
            self.fx, self.fy = self.camera_intrinsics[0, 0], self.camera_intrinsics[1, 1]
            self.cx, self.cy = self.camera_intrinsics[0, 2], self.camera_intrinsics[1, 2]
            self.get_logger().info(f"Calibrated camera: fx={self.fx}, fy={self.fy}")

    def depth_callback(self, msg):
        try: self.latest_depth_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e: self.get_logger().error(f"Depth conversion error: {e}")

    def pose_callback(self, msg):
        self.latest_robot_pose = msg.pose.pose 

    def publish_goal(self, target_pos, robot_pos):
        """ Handles the MIN_MOVE_THRESHOLD logic internally before publishing """
        if self.last_sent_goal is not None:
            dist = math.sqrt((target_pos[0] - self.last_sent_goal[0])**2 + (target_pos[1] - self.last_sent_goal[1])**2)
            if dist < self.MIN_MOVE_THRESHOLD:
                self.get_logger().info(f"Movement too small ({dist:.2f}m). Skipping update.")
                return 

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"

        msg.pose.position.x = float(target_pos[0])
        msg.pose.position.y = float(target_pos[1])
        msg.pose.position.z = 0.0

        dx = target_pos[0] - robot_pos[0]
        dy = target_pos[1] - robot_pos[1]   
        yaw = math.atan2(dy, dx)
        
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)

        self.goal_publisher.publish(msg)
        self.last_sent_goal = target_pos
        self.get_logger().info(f"New Goal published! Pos: ({target_pos[0]:.2f}, {target_pos[1]:.2f})")

    def tracking_callback(self, msg):
        if self.latest_depth_img is None or self.latest_robot_pose is None or self.camera_intrinsics is None:
            return
        
        # --- NEW LOGIC: Map Bounds Sanity Check ---
        if abs(self.latest_robot_pose.position.x) > self.MAP_BOUNDS or abs(self.latest_robot_pose.position.y) > self.MAP_BOUNDS:
            self.get_logger().error("SANITY CHECK: Robot out of map bounds! Stopping.")
            self.stop_robot()
            return
        
        target_person = next((p for p in msg.people if p.is_target), None)
        
        if target_person is None:
            if not self.target_was_lost:
                self.get_logger().info(f"Target ID {self.target_id} not found.")
                self.target_was_lost = True
            return

        self.last_target_time = self.get_clock().now()
        self.is_stopped_by_watchdog = False
        self.target_was_lost = False

        u, v = int(target_person.x_center), int(target_person.y_center)
        img_h, img_w = self.latest_depth_img.shape
        if u >= img_w or v >= img_h or u < 0 or v < 0: return

        r = 2
        v_min, v_max = max(0, v - r), min(img_h, v + r + 1)
        u_min, u_max = max(0, u - r), min(img_w, u + r + 1)
        roi = self.latest_depth_img[v_min:v_max, u_min:u_max]

        valid_pixels = roi[roi > 0]
        if len(valid_pixels) > 0:
            z_depth = float(np.median(valid_pixels)) / 1000
        else:
            self.get_logger().warn("Invalid depth.", throttle_duration_sec=1.0)
            return

        # Rifiutiamo valori negativi, NaN, o oltre i 3.0 metri
        if z_depth <= 0 or z_depth > 4.0 or np.isnan(z_depth):
            self.get_logger().warn(f"Depth {z_depth:.2f}m is invalid or too far. Ignored.")
            return

        x_cam = (u - self.cx) * z_depth / self.fx
        y_cam = (v - self.cy) * z_depth / self.fy
        
        q = self.latest_robot_pose.orientation
        rot_matrix = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        robot_pos = np.array([self.latest_robot_pose.position.x, self.latest_robot_pose.position.y, self.latest_robot_pose.position.z])
        person_in_camera_frame = np.array([z_depth, -x_cam, -y_cam])
        person_global_pos = np.dot(rot_matrix, person_in_camera_frame) + robot_pos

        distance_error = np.linalg.norm(person_global_pos[:2] - robot_pos[:2])

        self.get_logger().info(
            f"\nComputations:\n"
            f"Robot Pos: {robot_pos[:2]}\n"
            f"Person (Cam): {person_in_camera_frame}\n"
            f"Person (World): {person_global_pos}\n"
            f"Distance (Error): {distance_error:.2f} meter"
        )

        # --- NEW LOGIC: Hysteresis and Safe Goal ---
        if distance_error > self.SAFE_DISTANCE + 0.2: 
            direction_vector = (person_global_pos[:2] - robot_pos[:2]) / distance_error
            safe_goal_pos = person_global_pos[:2] - (direction_vector * self.SAFE_DISTANCE)
            
            # The movement threshold logic is now handled entirely inside publish_goal
            self.publish_goal(safe_goal_pos, robot_pos[:2])

        elif distance_error < self.SAFE_DISTANCE:
            self.get_logger().info("Target close. Stopping...")
            self.stop_robot()

    def watchdog_callback(self):
        now = self.get_clock().now()
        elapsed_time = (now - self.last_target_time).nanoseconds / 1e9

        if elapsed_time > self.TARGET_TIMEOUT_SEC and not self.is_stopped_by_watchdog:
            self.get_logger().warn(f"Watchdog triggered! Target lost for {elapsed_time:.1f}s. Stopping.")
            self.stop_robot()
            self.is_stopped_by_watchdog = True

def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()