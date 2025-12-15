import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from tracking_msgs.msg import PersonTrackerArray 
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
import numpy as np
import message_filters
from scipy.spatial.transform import Rotation as R

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')

        # variables
        self.target_id = 1  # ID of the tracked person
        self.camera_intrinsics = None # matrix K
        self.latest_depth_img = None
        self.latest_robot_pose = None
        
        self.bridge = CvBridge()

        ## SUBSCRIBERS
        #self.create_subscription(CameraInfo, '/camera/camera_info', self.info_callback, 10)
        #self.create_subscription(PersonTrackerArray, '/tracking_pkg/person_array', self.tracking_callback, 10)
        #self.create_subscription(Image, '/depth/rect_metric', self.depth_callback, 10)
        #self.create_subscription(PoseStamped, '/stella_slam/camera_pose', self.pose_callback, 10)
        
        self.fake_person_relative = np.array([2.0, 0.5, 0.0]) 
        self.get_logger().info(f"MODALITÀ TEST: Simulo persona a {self.fake_person_relative} (Relativo)")

        self.create_subscription(Odometry, '/odom', self.test_callback, 10) #for test

        self.get_logger().info("Controller Node Started. Waiting for data...")

    def test_callback(self, msg):
        """
        Questa funzione sostituisce tutto il calcolo della visione.
        Prende la posizione del robot da Gazebo e calcola dove si trova la persona finta nel mondo.
        """
        # 1. Estrai Posa Robot (World Frame) da Odometry
        # Nota: In /odom la posa è annidata dentro msg.pose.pose
        pos = msg.pose.pose.position
        orient = msg.pose.pose.orientation

        robot_pos_world = np.array([pos.x, pos.y, pos.z])
        
        # 2. Converti Quaternione (Orientamento) in Matrice di Rotazione
        r = R.from_quat([orient.x, orient.y, orient.z, orient.w])
        rotation_matrix = r.as_matrix()

        # 3. MATEMATICA DI TRASFORMAZIONE (Il cuore del tuo controller)
        # Formula: P_world = (Rotazione_Robot * P_relative) + Posizione_Robot
        
        # A. Ruota il punto relativo: Se il robot gira, il punto "davanti" a lui cambia direzione nel mondo
        person_rotated = np.dot(rotation_matrix, self.fake_person_relative)
        
        # B. Trasla: Aggiungi la posizione attuale del robot nella mappa
        person_world_pos = person_rotated + robot_pos_world

        # 4. CALCOLO DISTANZA (Verifica)
        # La distanza euclidea 2D. Se la persona è fissa relativamente, questa distanza dovrebbe essere costante (~2.06m)
        distance = np.linalg.norm(person_world_pos[:2] - robot_pos_world[:2])

        # --- LOG DI VERIFICA ---
        # Yaw (Rotazione sull'asse Z) in gradi per capire meglio come è girato il robot
        yaw_deg = r.as_euler('xyz', degrees=True)[2]

        print(f"--- TEST MATEMATICA ---")
        print(f"Robot (World):  X={robot_pos_world[0]:.2f}, Y={robot_pos_world[1]:.2f}, Angolo={yaw_deg:.1f}°")
        print(f"Persona (World): X={person_world_pos[0]:.2f}, Y={person_world_pos[1]:.2f}")
        print(f"Distanza Calcolata: {distance:.2f}m")
        print("-----------------------")

    '''
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
        self.latest_robot_pose = msg.pose

    def tracking_callback(self, msg):
        """ 
        This is the main function. It's called when the tracker sees someone.
        This is where the data fusion takes place.
        """
        if self.latest_depth_img is None or self.latest_robot_pose is None or self.camera_intrinsics is None:
            self.get_logger().warn("Waiting for Depth, Robot Pose, or Calibration...")
            return

        # 1. Search for the person with the target ID
        target_person = None
        for person in msg.people:
            if person.id == self.target_id:
                target_person = person
                break
        
        if target_person is None:
            self.get_logger().info(f"Target ID {self.target_id} not found.")
            return

        # 2. Extract pixel coordinates (u,v) from tracking message
        u = int(target_person.x_center)
        v = int(target_person.y_center)

        # Image boundary check (to avoid crashing if u,v are out of bounds)
        h_img, w_img = self.latest_depth_img.shape
        if u >= w_img or v >= h_img or u < 0 or v < 0:
            return

        # 3. Get the Z depth at that pixel
        z_depth = self.latest_depth_img[v, u] 

        if z_depth <= 0 or np.isnan(z_depth):
            self.get_logger().warn("Invalid depth for target.")
            return

        # 4. CALCULATE POSITION RELATIVE TO THE CAMERA (X_cam, Y_cam, Z_cam)
        # Pinhole Camera Formulas
        x_cam = (u - self.cx) * z_depth / self.fx
        y_cam = (v - self.cy) * z_depth / self.fy
        z_cam = z_depth 

        # Person position vector relative to the camera
        person_in_camera_frame = np.array([x_cam, y_cam, z_cam])

        # 5. GLOBAL POSITION CALCULATION (Map)
        # Transform the robot's pose (quaternion) into a rotation matrix
        q = self.latest_robot_pose.orientation
        rot_matrix = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        
        # Robot position (translation)
        robot_pos = np.array([self.latest_robot_pose.position.x, 
                              self.latest_robot_pose.position.y, 
                              self.latest_robot_pose.position.z])

        # Final transformation: P_world = R * P_cam + T_robot
        person_global_pos = np.dot(rot_matrix, person_in_camera_frame) + robot_pos

        # 6. ERROR CALCULATION (Robot - Person Distance)
        distance_error = np.linalg.norm(person_global_pos[:2] - robot_pos[:2])

        self.get_logger().info(
            f"\nComputations:\n"
            f"Robot Pos: {robot_pos[:2]}\n"
            f"Person (Cam): {person_in_camera_frame}\n"
            f"Person (World): {person_global_pos}\n"
            f"Distance (Errore): {distance_error:.2f} meter"
        )
        
        self.publish_goal(person_global_pos)
    '''
def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()