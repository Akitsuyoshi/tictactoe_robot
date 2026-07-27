#!/usr/bin/env python3
from ultralytics import YOLO
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from tictactoe_interfaces.msg import InferenceResult, YoloInference

bridge = CvBridge()


class ObjectDetection(Node):
    def __init__(self) -> None:
        super().__init__("object_detection")

        self.detection_model = YOLO(
            "/home/user/ros2_ws/src/tictactoe_robot/board_perception/weights/best.onnx",
            task="detect"
        )
        print(f"Model is loaded, class: ${self.detection_model.names}")
        
        self.yolo_inference = YoloInference()

        self.subscription = self.create_subscription(
            Image,
            "/camera1/image_raw",
            self.camera_callback,
            10
        )
        
        self.yolo_pub = self.create_publisher(YoloInference, "/yolo_inference", 1)
        self.img_pub = self.create_publisher(Image, "/inference_result", 1)

    
    def camera_callback(self, msg: Image) -> None:
        img = bridge.imgmsg_to_cv2(msg, "bgr8")
        results = self.detection_model(img)

        self.yolo_inference.header = msg.header

        for r in results:
            for box in r.boxes:
                self.inf_result = InferenceResult()

                x1, y1, x2, y2 = map(int, box.xyxy[0])
                class_id = int(box.cls[0])
                confidence = float(box.conf[0])

                self.inf_result.class_name = self.detection_model.names[class_id]
                self.inf_result.confidence = confidence

                self.inf_result.left = x1
                self.inf_result.top = y1
                self.inf_result.right = x2
                self.inf_result.bottom = y2

                self.inf_result.box_width = x2 - x1
                self.inf_result.box_height = y2 - y1

                self.inf_result.x = x1 + self.inf_result.box_width / 2.0
                self.inf_result.y = y1 + self.inf_result.box_height / 2.0

                self.yolo_inference.detections.append(self.inf_result)

        # Publish annotated image
        annotated_frame = results[0].plot()
        img_msg = bridge.cv2_to_imgmsg(annotated_frame)
        img_msg.header = msg.header
        self.img_pub.publish(img_msg)

        # Publish object detection inferece
        self.yolo_pub.publish(self.yolo_inference)
        self.yolo_inference.detections.clear()


def main(args=None) -> None:
    rclpy.init(args=args)
    object_detection = ObjectDetection()
    rclpy.spin(object_detection)
    object_detection.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()