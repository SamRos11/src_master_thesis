import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from tracking_msgs.msg import PersonTracker, PersonTrackerArray
from cv_bridge import CvBridge
from ultralytics import YOLO
from deep_sort_realtime.deepsort_tracker import DeepSort
import cv2


class TrackingNode(Node):
    def __init__(self):
        super().__init__('tracking_node')
        self.bridge = CvBridge()

        # Subscriber: questo riceve già i frame della webcam da ROS
        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            10
        )

        # Publisher: manda fuori i risultati del tracking
        self.publisher = self.create_publisher(
            PersonTrackerArray,
            '/tracking/people',
            10
        )

        self.get_logger().info("YOLO + DeepSORT tracking node started.")

        # Caricamento YOLO
        self.model = YOLO("yolo11s.pt")

        # DeepSORT setup
        self.tracker = DeepSort(
            max_iou_distance=0.9,
            max_age=120,
            n_init=3,
            max_cosine_distance=0.6,
            nn_budget=30
        )

    def image_callback(self, msg):
        # Converti ROS -> OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # YOLO detection
        results = self.model(frame, verbose=False, classes=[0], conf=0.7)[0]

        detections = []
        for box in results.boxes:
            x1, y1, x2, y2 = box.xyxy[0]
            conf = float(box.conf[0])
            detections.append([[x1, y1, x2 - x1, y2 - y1], conf, 'person'])

        # DeepSORT tracking
        tracks = self.tracker.update_tracks(detections, frame=frame)

        # Messaggio ROS da pubblicare
        array_msg = PersonTrackerArray()

        # Disegno e creazione array messaggi
        for track in tracks:
            if not track.is_confirmed():
                continue

            x1, y1, x2, y2 = map(int, track.to_ltrb())
            track_id = track.track_id

            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, f"ID {track_id}", (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            p = PersonTracker()
            p.id = int(track_id)
            p.x_center = float((x1 + x2) / 2)
            p.y_center = float((y1 + y2) / 2)
            p.w = float(x2 - x1)
            p.h = float(y2 - y1)
            array_msg.people.append(p)

        # Pubblica i risultati
        self.publisher.publish(array_msg)

        # Mostra finestra con risultati
        cv2.imshow("YOLO + DeepSORT Tracking", frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = TrackingNode()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == '__main__':
    main()