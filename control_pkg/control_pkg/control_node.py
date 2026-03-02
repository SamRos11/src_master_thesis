import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from tracking_msgs.msg import PersonTrackerArray 
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
import numpy as np
import message_filters
from scipy.spatial.transform import Rotation as R
from rclpy.qos import qos_profile_sensor_data
import math

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        
        # Variables
        self.target_id = 1  # ID of the tracked person
        self.camera_intrinsics = None # matrix K
        self.latest_depth_img = None
        self.latest_robot_pose = None
        self.last_goal = [0.0, 0.0]
        self.MIN_MOVE_THRESHOLD = 0.30
        self.SAFE_DISTANCE = 0.5
        
        self.bridge = CvBridge()
        self.target_was_lost = False

        ## SUBSCRIBERS
        self.create_subscription(CameraInfo, '/camera/camera/color/camera_info', self.info_callback, 10)
        self.create_subscription(PersonTrackerArray, '/tracking/people', self.tracking_callback, 10)
        self.create_subscription(Odometry, '/run_slam/camera_pose', self.pose_callback, qos_profile_sensor_data)
        self.create_subscription(
            Image, 
            '/camera/camera/aligned_depth_to_color/image_raw', 
            self.depth_callback, 
            qos_profile_sensor_data
        )

        self.goal_publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)

    def info_callback(self, msg):
        """ Reads the camera's intrinsic matrix (fx, fy, cx, cy) """
        if self.camera_intrinsics is None:
            self.camera_intrinsics = np.array(msg.k).reshape((3, 3))
            self.fx = self.camera_intrinsics[0, 0]
            self.fy = self.camera_intrinsics[1, 1]
            self.cx = self.camera_intrinsics[0, 2]
            self.cy = self.camera_intrinsics[1, 2]
            self.get_logger().info(f"Calibrated camera: fx={self.fx}, fy={self.fy}")

    def depth_callback(self, msg):
        """ Save the last available depth image """
        try:
            self.latest_depth_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f"Depth conversion error: {e}")

    def pose_callback(self, msg):
        """ Save the robot's last known position from Stella VSLAM """
        self.latest_robot_pose = msg.pose.pose 
        
        #self.get_logger().info(f"Pose ricevuta! X: {self.latest_robot_pose.position.x:.2f}")

    def publish_goal(self, target_pos, robot_pos):
        msg = PoseStamped()
        
        # Header obbligatorio
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"  # Le coordinate sono globali (riferite alla mappa)

        # Calculated position
        msg.pose.position.x = float(target_pos[0])
        msg.pose.position.y = float(target_pos[1])
        msg.pose.position.z = 0.0  # Nav2 works in 2D

        dx = target_pos[0] - robot_pos[0]
        dy = target_pos[1] - robot_pos[1]   
        
        # Calculate the angle in radians (Yaw)
        yaw = math.atan2(dy, dx)
        
        # Convert Yaw to Quaternion (Z-axis only)
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)

        # Post to topic '/goal_pose'
        self.goal_publisher.publish(msg)

    def tracking_callback(self, msg):
        """ 
        This is the main function. It's called when the tracker sees someone.
        This is where the data fusion takes place.
        """
        if self.latest_depth_img is None:
            self.get_logger().warn("Waiting for Depth image...", throttle_duration_sec=2.0)
            return
        
        if self.latest_robot_pose is None:
            self.get_logger().warn("Waiting for Robot Pose (SLAM)...", throttle_duration_sec=2.0)
            return
            
        if self.camera_intrinsics is None:
            self.get_logger().warn("Waiting for Camera Calibration...", throttle_duration_sec=2.0)
            return
        
        # Search for the person with the target ID
        target_person = None
        
        for person in msg.people:
            if person.is_target: 
                target_person = person
                break
        
        if target_person is None:
            if not self.target_was_lost:
                self.get_logger().info(f"Target ID {self.target_id} not found.")
                self.target_was_lost = True
            return

        # Extract pixel coordinates (u,v) from tracking message
        u = int(target_person.x_center)
        v = int(target_person.y_center)
        h = int(target_person.h)

        v_offset = int(v + (h * 0.25))
        
        v = v_offset

        # Image boundary check (to avoid crashing if u,v are out of bounds)
        h_img, w_img = self.latest_depth_img.shape
        if u >= w_img or v >= h_img or u < 0 or v < 0:
            return

        # Get the Z depth at that pixel
        
        # Define the radius (5x5 means radius 2: center + 2 on each side)
        r = 2
        img_h, img_w = self.latest_depth_img.shape


        # Calculate safe bounds (to avoid going out of image boundaries and crashing)
        v_min = max(0, v - r)
        v_max = min(img_h, v + r + 1)
        u_min = max(0, u - r)
        u_max = min(img_w, u + r + 1)

        # Extract the small square (ROI - Region of Interest)
        roi = self.latest_depth_img[v_min:v_max, u_min:u_max]

        # Filter only valid pixels (greater than 0)
        valid_pixels = roi[roi > 0]

        # Calculate the median
        if len(valid_pixels) > 0:
            z_depth = float(np.median(valid_pixels)) / 1000
        else:
            self.get_logger().warn("Invalid depth.", throttle_duration_sec=1.0)
            z_depth = 0.0   
            return

        if z_depth <= 0 or np.isnan(z_depth):
            self.get_logger().warn("Invalid depth for target.")
            return

        # CALCULATE POSITION RELATIVE TO THE CAMERA (X_cam, Y_cam, Z_cam)
        # Pinhole Camera Formulas
        x_cam = (u - self.cx) * z_depth / self.fx
        y_cam = (v - self.cy) * z_depth / self.fy
        z_cam = z_depth 

        # GLOBAL POSITION CALCULATION (Map)
        # Transform the robot's pose (quaternion) into a rotation matrix
        q = self.latest_robot_pose.orientation

        #rot_matrix = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        #only for temponary test
        rot_matrix = R.from_quat([0.0, 0.0, 0.0, 1.0]).as_matrix()

        # Robot position (translation)
        #robot_pos = np.array([self.latest_robot_pose.position.x, 
        #                      self.latest_robot_pose.position.y, 
        #                      self.latest_robot_pose.position.z])
        #only for temponary test
        robot_pos = np.array([0, 
                              0, 
                              0])
        
        # Person position vector relative to the camera
        person_in_camera_frame = np.array([z_cam, -x_cam, -y_cam])

        # Final transformation: P_world = R * P_cam + T_robot
        person_global_pos = np.dot(rot_matrix, person_in_camera_frame) + robot_pos

        # Handle the very first iteration where 'last_goal' might not exist yet
        if not hasattr(self, 'last_goal') or self.last_goal is None:
             self.last_goal = person_global_pos

        # ERROR CALCULATION (Robot - Person Distance)
        distance_error = np.linalg.norm(person_global_pos[:2] - robot_pos[:2])

        

        if distance_error > self.SAFE_DISTANCE:
            # Calculate the unit vector pointing from Robot -> Person
            direction_vector = (person_global_pos[:2] - robot_pos[:2]) / distance_error
            
            # The actual goal is: Person Position minus the Safety Buffer along that vector
            # This ensures the goal is in front of the person, not inside them.
            safe_goal_pos = person_global_pos[:2] - (direction_vector * self.SAFE_DISTANCE)
        else:
            # If the person is too close (inside the buffer), stay put (Goal = Current Pos)
            safe_goal_pos = robot_pos[:2]

        self.get_logger().info(
            f"\nComputations:\n"
            f"Robot Pos: {robot_pos[:2]}\n"
            f"Person (Cam): {person_in_camera_frame}\n"
            f"Person (World): {person_global_pos}\n"
            f"Distance (Errore): {distance_error:.2f} meter"
        )

        dx = person_global_pos[0] - self.last_goal[0] 
        dy = person_global_pos[1] - self.last_goal[1]
        movement_delta = math.sqrt(dx**2 + dy**2)

        if movement_delta < self.MIN_MOVE_THRESHOLD:
            # If the movement is below the threshold (e.g., 0.30m), it's considered noise.
            # We RETURN here to avoid spamming the planner with identical goals.
            self.get_logger().info(f"Movement too small ({movement_delta:.2f}m). Skipping update.")
            return 
        else:
            # If valid movement is detected, publish the NEW SAFE GOAL.
            # We pass 'safe_goal_pos' to the navigator, not the raw person position.
            self.publish_goal(safe_goal_pos, robot_pos[:2])
            
            # Update the 'last_goal' memory with the CURRENT detected person position
            # so we can calculate the delta correctly in the next loop.
            self.last_goal = person_global_pos 
            self.get_logger().info(f"New Goal published! Delta: {movement_delta:.2f}m")


        
        

def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()