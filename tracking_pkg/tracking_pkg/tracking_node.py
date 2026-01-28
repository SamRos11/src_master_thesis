import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from tracking_msgs.msg import PersonTracker, PersonTrackerArray
from cv_bridge import CvBridge
from ultralytics import YOLO
from deep_sort_realtime.deepsort_tracker import DeepSort
import cv2
import numpy as np


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
        self.model = YOLO("yolo11s-pose.pt")

        self.locked_target_id = None

        # DeepSORT setup
        self.tracker = DeepSort(
            max_iou_distance=0.9,
            max_age=120,
            n_init=3,
            max_cosine_distance=0.6,
            nn_budget=30
        )

    def detect_gesture(self, kpts):
        """
        Determines the gesture state based on keypoints.
        COCO Keypoint Indices: 0: Nose, 9: Left Wrist, 10: Right Wrist
        
        Returns:
        - "UNLOCK": Both hands are above the nose (Stop following).
        - "LOCK": One hand is above the nose (Start following).
        - "NONE": No specific gesture detected.
        """
        if kpts is None or len(kpts) == 0:
            return "NONE"
            
        # Extract specific keypoints
        nose = kpts[0]
        wrist_l = kpts[9]
        wrist_r = kpts[10]
        
        # Confidence threshold to ignore low-quality detections
        conf_threshold = 0.5
        
        # Check if wrists are valid and above the nose (lower Y value = higher in image)
        right_up = (wrist_r[2] > conf_threshold and nose[2] > conf_threshold and wrist_r[1] < nose[1])
        left_up = (wrist_l[2] > conf_threshold and nose[2] > conf_threshold and wrist_l[1] < nose[1])

        # Priority Logic: Both hands up overrides single hand
        if right_up and left_up:
            return "UNLOCK"
        elif right_up or left_up:
            return "LOCK"
        
        return "NONE"

    def image_callback(self, msg):
        # ROS -> OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # YOLO detection
        results = self.model(frame, verbose=False, classes=[0], conf=0.7)[0]

        detections = []

        yolo_boxes_centers = [] 
        yolo_keypoints = []

        if results.boxes:
            boxes_data = results.boxes.xyxy.cpu().numpy()
            conf_data = results.boxes.conf.cpu().numpy()
            
            # Extract keypoints if available
            if results.keypoints is not None:
                kpts_data = results.keypoints.data.cpu().numpy() # Shape: (N, 17, 3)
            else:
                kpts_data = [None] * len(boxes_data)

            for i, box in enumerate(boxes_data):
                x1, y1, x2, y2 = box
                w, h = x2 - x1, y2 - y1
                center_x, center_y = x1 + w/2, y1 + h/2
                
                detections.append([[x1, y1, w, h], float(conf_data[i]), 'person'])
                
                # Store data for matching keypoints to tracks later
                yolo_boxes_centers.append((center_x, center_y))
                yolo_keypoints.append(kpts_data[i])

        # DeepSORT Tracking
        tracks = self.tracker.update_tracks(detections, frame=frame)

        array_msg = PersonTrackerArray()

        for track in tracks:
            if not track.is_confirmed():
                continue

            x1, y1, x2, y2 = map(int, track.to_ltrb())
            track_id = track.track_id

            # MATCH TRACK TO KEYPOINTS
            # DeepSORT returns tracks, but we need the original keypoints from YOLO.
            # We find the closest original YOLO box to the current DeepSORT track center.
            current_kpts = None
            
            track_center_x = (x1 + x2) / 2
            track_center_y = (y1 + y2) / 2
            
            min_dist = 100.0 # Max pixel distance for valid match
            best_idx = -1
            
            for i, (yx, yy) in enumerate(yolo_boxes_centers):
                dist = np.sqrt((track_center_x - yx)**2 + (track_center_y - yy)**2)
                if dist < min_dist:
                    min_dist = dist
                    best_idx = i
            
            if best_idx != -1:
                current_kpts = yolo_keypoints[best_idx]

            # GESTURE LOGIC (LOCK / UNLOCK)
            gesture = self.detect_gesture(current_kpts)

            if gesture == "UNLOCK":
                # Only unlock if the person gesturing is the current target
                if self.locked_target_id == track_id:
                    self.locked_target_id = None
                    self.get_logger().info(f"ID {track_id} gestured UNLOCK. Target disengaged.")
                
                # Visual feedback on screen
                cv2.putText(frame, "UNLOCKING...", (x1, y1 - 40), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            elif gesture == "LOCK":
                # Set this person as the new target
                self.locked_target_id = track_id
                self.get_logger().info(f"ID {track_id} gestured LOCK. Target engaged.")
                # Visual feedback on screen
                cv2.putText(frame, "LOCKED!", (x1, y1 - 40), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
            # PREPARE ROS MESSAGE
            p = PersonTracker()
            p.id = int(track_id)
            p.x_center = float((x1 + x2) / 2)
            p.y_center = float((y1 + y2) / 2)
            p.w = float(x2 - x1)
            p.h = float(y2 - y1)

            # Set the flag for the Control Node
            if self.locked_target_id is not None and track_id == self.locked_target_id:
                p.is_target = True 
                color = (0, 0, 255) # Red for Target
                label = f"ID {track_id} [TARGET]"
            else:
                p.is_target = False
                color = (0, 255, 0) # Green for others
                label = f"ID {track_id}"

            array_msg.people.append(p)

            # Draw bounding box and label
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            cv2.putText(frame, label, (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        # Publish the array
        self.publisher.publish(array_msg)

        # Display the result
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