import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
 
 
class PersonDetectorNode(Node):
    def __init__(self):
        super().__init__('person_detector_node')
 
        # --- Parameters (override via launch file or --ros-args -p) ---
        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.declare_parameter('weights_path', 'yolov4-tiny.weights')
        self.declare_parameter('config_path', 'yolov4-tiny.cfg')
        self.declare_parameter('names_path', 'coco.names')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('nms_threshold', 0.4)
        self.declare_parameter('detection_hold_frames', 5)  # debounce window
        self.declare_parameter('publish_annotated', True)
 
        camera_topic = self.get_parameter('camera_topic').value
        weights = self.get_parameter('weights_path').value
        config = self.get_parameter('config_path').value
        names_path = self.get_parameter('names_path').value
        self.conf_thresh = self.get_parameter('confidence_threshold').value
        self.nms_thresh = self.get_parameter('nms_threshold').value
        self.hold_frames = self.get_parameter('detection_hold_frames').value
        self.publish_annotated = self.get_parameter('publish_annotated').value
 
        # --- Load COCO class names, find the "person" class index ---
        with open(names_path, 'r') as f:
            self.classes = [line.strip() for line in f.readlines()]
        self.person_class_id = self.classes.index('person')
 
        # --- Load the YOLO network ---
        self.net = cv2.dnn.readNetFromDarknet(config, weights)
        self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
        self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)  # -> DNN_TARGET_CUDA if you have GPU support built
        self.output_layers = self.net.getUnconnectedOutLayersNames()
 
        self.bridge = CvBridge()
 
        # --- Debounce state: require N consecutive frames before flipping ---
        # This stops a single blurry/occluded frame from firing (or clearing)
        # the wave trigger.
        self.consecutive_detections = 0
        self.consecutive_misses = 0
        self.person_present = False
 
        # --- ROS interfaces ---
        self.sub = self.create_subscription(Image, camera_topic, self.image_callback, 10)
        self.detection_pub = self.create_publisher(Bool, '/person_detected', 10)
        if self.publish_annotated:
            self.annotated_pub = self.create_publisher(Image, '/person_detection/annotated', 10)
 
        self.get_logger().info(f'Person detector ready, subscribed to {camera_topic}')
 
    def image_callback(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        height, width = frame.shape[:2]
 
        blob = cv2.dnn.blobFromImage(frame, 1 / 255.0, (416, 416), swapRB=True, crop=False)
        self.net.setInput(blob)
        outputs = self.net.forward(self.output_layers)
 
        boxes, confidences = [], []
        for output in outputs:
            for detection in output:
                scores = detection[5:]
                class_id = int(np.argmax(scores))
                confidence = float(scores[class_id])
                if class_id == self.person_class_id and confidence > self.conf_thresh:
                    cx, cy, w, h = (detection[0:4] * np.array([width, height, width, height])).astype(int)
                    x = int(cx - w / 2)
                    y = int(cy - h / 2)
                    boxes.append([x, y, int(w), int(h)])
                    confidences.append(confidence)
 
        indices = cv2.dnn.NMSBoxes(boxes, confidences, self.conf_thresh, self.nms_thresh)
        person_found_this_frame = len(indices) > 0
 
        # --- Debounce logic ---
        if person_found_this_frame:
            self.consecutive_detections += 1
            self.consecutive_misses = 0
        else:
            self.consecutive_misses += 1
            self.consecutive_detections = 0
 
        if not self.person_present and self.consecutive_detections >= self.hold_frames:
            self.person_present = True
            self.get_logger().info('Person detected')
        elif self.person_present and self.consecutive_misses >= self.hold_frames:
            self.person_present = False
            self.get_logger().info('Person no longer detected')
 
        self.detection_pub.publish(Bool(data=self.person_present))
 
        if self.publish_annotated:
            annotated = frame.copy()
            flat_indices = indices.flatten() if len(indices) else []
            for i in flat_indices:
                x, y, w, h = boxes[i]
                cv2.rectangle(annotated, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.putText(annotated, f'person {confidences[i]:.2f}', (x, max(y - 10, 0)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            self.annotated_pub.publish(self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8'))
 
 
def main(args=None):
    rclpy.init(args=args)
    node = PersonDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
 
 
if __name__ == '__main__':
    main()
 