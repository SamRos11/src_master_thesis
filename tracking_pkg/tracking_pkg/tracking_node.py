import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from tracking_msgs.msg import PersonTracker, PersonTrackerArray
from cv_bridge import CvBridge
from ultralytics import YOLO
from deep_sort_realtime.deepsort_tracker import DeepSort
import cv2
import numpy as np
import torch

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

        # Upload of YOLO's model
        self.model = YOLO("yolo11n-pose.pt")

        # Use of GPU
        self.model.to('cuda')

        self.locked_target_id = None

        # DeepSORT setup
        self.tracker = DeepSort(
            max_iou_distance=0.9,
            max_age=90,
            n_init=3,
            max_cosine_distance=0.6,
            nn_budget=30,
            embedder_gpu=True # Use of GPU
        )

    def detect_gesture(self, kpts):
        """
        Determines the gesture state based on keypoints.
        COCO Keypoint Indices: 0: Nose, 9: Left Wrist, 10: Right Wrist
        """
        if kpts is None or len(kpts) == 0:
            return "NONE"
            
        nose = kpts[0]
        wrist_l = kpts[9]
        wrist_r = kpts[10]
        
        conf_threshold = 0.5
        
        right_up = (wrist_r[2] > conf_threshold and nose[2] > conf_threshold and wrist_r[1] < nose[1])
        left_up = (wrist_l[2] > conf_threshold and nose[2] > conf_threshold and wrist_l[1] < nose[1])

        if right_up and left_up:
            return "UNLOCK"
        elif right_up or left_up:
            return "LOCK"
        
        return "NONE"

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # YOLO detection (imgsz=320 optimizes memory usage on Jetson)
        results = self.model(frame, verbose=False, classes=[0], conf=0.7, device='cuda:0', imgsz=320, half=True)[0]

        detections = []
        yolo_boxes_centers = [] 
        yolo_keypoints = []

        if results.boxes:
            boxes_data = results.boxes.xyxy.cpu().numpy()
            conf_data = results.boxes.conf.cpu().numpy()
            
            if results.keypoints is not None:
                kpts_data = results.keypoints.data.cpu().numpy()
            else:
                kpts_data = [None] * len(boxes_data)

            for i, box in enumerate(boxes_data):
                x1, y1, x2, y2 = box
                w, h = x2 - x1, y2 - y1
                
                # --- FILTRO GEOMETRICO ---
                # Ignora target troppo piccoli (lontani) o troppo larghi rispetto all'altezza (falsi positivi)
                if h < 120 or w < 40 or w > h * 1.5:
                    continue
                # -------------------------

                center_x, center_y = x1 + w/2, y1 + h/2
                detections.append([[x1, y1, w, h], float(conf_data[i]), 'person'])
                
                yolo_boxes_centers.append((center_x, center_y))
                yolo_keypoints.append(kpts_data[i])

        tracks = self.tracker.update_tracks(detections, frame=frame)
        array_msg = PersonTrackerArray()

        for track in tracks:
            if not track.is_confirmed():
                continue
            if track.time_since_update > 1:
                continue

            x1, y1, x2, y2 = map(int, track.to_ltrb())
            track_id = track.track_id

            current_kpts = None
            track_center_x = (x1 + x2) / 2
            track_center_y = (y1 + y2) / 2
            
            min_dist = 100.0 
            best_idx = -1
            
            for i, (yx, yy) in enumerate(yolo_boxes_centers):
                dist = np.sqrt((track_center_x - yx)**2 + (track_center_y - yy)**2)
                if dist < min_dist:
                    min_dist = dist
                    best_idx = i
            
            if best_idx != -1:
                current_kpts = yolo_keypoints[best_idx]

            gesture = self.detect_gesture(current_kpts)

            if gesture == "UNLOCK":
                if self.locked_target_id == track_id:
                    self.locked_target_id = None
                    self.get_logger().info(f"ID {track_id} gestured UNLOCK. Target disengaged.")
                cv2.putText(frame, "UNLOCKING...", (x1, y1 - 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            elif gesture == "LOCK":
                self.locked_target_id = track_id
                self.get_logger().info(f"ID {track_id} gestured LOCK. Target engaged.")
                cv2.putText(frame, "LOCKED!", (x1, y1 - 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
            p = PersonTracker()
            p.id = int(track_id)
            p.x_center = float((x1 + x2) / 2)
            p.y_center = float((y1 + y2) / 2)
            p.w = float(x2 - x1)
            p.h = float(y2 - y1)

            if self.locked_target_id is not None and track_id == self.locked_target_id:
                p.is_target = True 
                color = (0, 0, 255) 
                label = f"ID {track_id} [TARGET]"
            else:
                p.is_target = False
                color = (0, 255, 0) 
                label = f"ID {track_id}"

            array_msg.people.append(p)
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        self.publisher.publish(array_msg)
        torch.cuda.empty_cache()

def main(args=None):
    rclpy.init(args=args)
    node = TrackingNode()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()