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

# NEW IMPORTS FOR NAV2 ACTIONS
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        
        self.target_id = 1
        self.camera_intrinsics = None
        self.latest_depth_img = None
        self.latest_robot_pose = None
        self.last_sent_goal = None
        
        # Navigation Thresholds
        self.MIN_MOVE_THRESHOLD = 0.3
        self.MAX_JUMP_THRESHOLD = 4.0
        self.SAFE_DISTANCE = 1.8
        self.MAP_BOUNDS = 29.5 
        
        self.bridge = CvBridge()
        self.target_was_lost = False

        # Watchdog setup
        self.TARGET_TIMEOUT_SEC = 2.5
        self.last_target_time = self.get_clock().now()
        self.is_stopped_by_watchdog = False
        
        self.watchdog_timer = self.create_timer(0.5, self.watchdog_callback)

        # Subscribers
        self.create_subscription(CameraInfo, '/camera/camera/color/camera_info', self.info_callback, 10)
        self.create_subscription(PersonTrackerArray, '/tracking/people', self.tracking_callback, 10)
        self.create_subscription(Odometry, '/odometry/global', self.pose_callback, qos_profile_sensor_data)
        self.create_subscription(Image, '/camera/camera/aligned_depth_to_color/image_raw', self.depth_callback, qos_profile_sensor_data)

        # Standard Publisher for immediate stop
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # ACTION CLIENT FOR NAV2
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.current_goal_handle = None

    def stop_robot(self):
        """ 
        Cancels the active Nav2 goal and issues an immediate stop command.
        This prevents Nav2 and the control node from fighting over /cmd_vel.
        """
        # Step 1: Tell Nav2 to abort the current path and stop recoveries
        if self.current_goal_handle is not None:
            self.get_logger().info("Cancelling active Nav2 Goal...")
            self.current_goal_handle.cancel_goal_async()
            self.current_goal_handle = None

        # Step 2: Send physical stop command to the motors
        cmd = Twist()
        self.cmd_vel_publisher.publish(cmd)
        self.get_logger().info("Robot stopped.", once=True)

    def info_callback(self, msg):
        if self.camera_intrinsics is None:
            self.camera_intrinsics = np.array(msg.k).reshape((3, 3))
            self.fx, self.fy = self.camera_intrinsics[0, 0], self.camera_intrinsics[1, 1]
            self.cx, self.cy = self.camera_intrinsics[0, 2], self.camera_intrinsics[1, 2]

    def depth_callback(self, msg):
        try: self.latest_depth_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e: self.get_logger().error(f"Depth conversion error: {e}")

    def pose_callback(self, msg):
        self.latest_robot_pose = msg.pose.pose 

    def publish_goal(self, target_pos, robot_pos):
        """ 
        Filters the goal based on movement and sends it asynchronously to the Nav2 Action Server.
        """
        # Filter 1: Minimum Movement Threshold
        if self.last_sent_goal is not None:
            dist = math.sqrt((target_pos[0] - self.last_sent_goal[0])**2 + (target_pos[1] - self.last_sent_goal[1])**2)
            if dist < self.MIN_MOVE_THRESHOLD:
                self.get_logger().info(f"Movement too small ({dist:.2f}m). Skipping update.")
                return 

        # Prepare the Action Goal message
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.header.frame_id = "map"

        goal_msg.pose.pose.position.x = float(target_pos[0])
        goal_msg.pose.pose.position.y = float(target_pos[1])
        goal_msg.pose.pose.position.z = 0.0

        if self.last_sent_goal is not None:
            # Calcola l'angolo in base allo spostamento della persona (da dove era a dove è ora)
            dx_move = target_pos[0] - self.last_sent_goal[0]
            dy_move = target_pos[1] - self.last_sent_goal[1]
            yaw = math.atan2(dy_move, dx_move)
        else:
            # Fallback per il primo goal in assoluto
            dx_robot = target_pos[0] - robot_pos[0]
            dy_robot = target_pos[1] - robot_pos[1]
            yaw = math.atan2(dy_robot, dx_robot)
        
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        # Wait for the Action Server to be available and send the goal
        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error("Nav2 Action Server not available!")
            return

        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

        self.last_sent_goal = target_pos
        self.get_logger().info(f"New Action Goal sent! Pos: ({target_pos[0]:.2f}, {target_pos[1]:.2f})")

    def goal_response_callback(self, future):
        """ Callback executed when the Action Server accepts or rejects the goal """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected by Nav2')
            return
        
        # Save the active goal handle so it can be cancelled later if needed
        self.current_goal_handle = goal_handle

    def tracking_callback(self, msg):
        if self.latest_depth_img is None or self.latest_robot_pose is None or self.camera_intrinsics is None:
            return
        
        # Sanity Check Boundary
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

        # Evaluate if the target was lost in the previous execution loop
        if self.target_was_lost:
            self.get_logger().info("Target re-engaged! Resetting jump detection memory.")
            self.last_sent_goal = None

        # Target found: reset watchdog and update state flags
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

        # Depth limits filter
        if z_depth <= 0 or z_depth > 5.0 or np.isnan(z_depth):
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
        
        direction_vector = (person_global_pos[:2] - robot_pos[:2]) / distance_error
        safe_goal_pos = person_global_pos[:2] - (direction_vector * self.SAFE_DISTANCE)
        
        # Protects navigation from immediate SLAM localization shifts
        if self.last_sent_goal is not None:
            jump_dist = np.linalg.norm(safe_goal_pos - np.array(self.last_sent_goal))
            if jump_dist > self.MAX_JUMP_THRESHOLD:
                self.get_logger().warn(f"JUMP DETECTED ({jump_dist:.2f}m). Ignoring erratic goal.")
                return 

        BUFFER = 0.2 

        if distance_error > self.SAFE_DISTANCE + BUFFER: 
            # Target is far: move forward
            self.publish_goal(safe_goal_pos, robot_pos[:2])
            
            '''elif distance_error < self.SAFE_DISTANCE - BUFFER:
            # Target is too close: trigger backward motion via Nav2 allow_reversing
            self.get_logger().info(f"Target too close ({distance_error:.2f}m)! Backing up...")
            self.publish_goal(safe_goal_pos, robot_pos[:2])'''
            
        else:
            # Target is within the deadband zone (1.6m - 2.0m)
            self.get_logger().info("Target in optimal zone. Stopping...", throttle_duration_sec=1.0)
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