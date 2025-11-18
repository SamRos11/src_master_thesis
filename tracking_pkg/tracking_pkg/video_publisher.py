import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class VideoPublisher(Node):
    def __init__(self):
        super().__init__('video_publisher')
        self.publisher_ = self.create_publisher(Image, '/camera/color/image_raw', 10)
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture('/home/samu/ros2_ws/src/tracking_pkg/tracking_pkg/Test_LS.mp4')
        if not self.cap.isOpened():
            self.get_logger().error("ERRORE FATALE: Impossibile trovare o aprire il file video.")
            # Solleva un'eccezione o esci per vedere l'errore nel terminale
            raise FileNotFoundError("Il file video specificato non è stato trovato o letto.")
            return

        self.timer = self.create_timer(0.03, self.timer_callback)
        self.get_logger().info('Video Publisher avviato su /camera/color/image_raw')

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            return
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = VideoPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()