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

        # Parameters
        self.declare_parameter("model_path", "")
        self.declare_parameter("image_topic", "/camera/D435/color/image_raw")
        model_path = self.get_parameter("model_path").value
        image_topic = self.get_parameter("image_topic").value


        self.detection_model = YOLO(model_path, task="detect")
        print(f"Model is loaded, class: ${self.detection_model.names}")

        self.latest_image = None
        self.latest_header = None
        
        self.subscription = self.create_subscription(
            Image,
            image_topic,
            self.camera_callback,
            1,
        )

        self.timer = self.create_timer(
            0.5,
            self.inference_timer,
        )
        
        self.yolo_pub = self.create_publisher(YoloInference, "/yolo_inference", 1)
        self.img_pub = self.create_publisher(Image, "/inference_result", 1)

    
    def camera_callback(self, msg: Image) -> None:
        self.latest_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        self.latest_header = msg.header


    def inference_timer(self) -> None:
        if self.latest_image is None:
            return
        
        results = self.detection_model(self.latest_image)
        inference_msg = YoloInference()
        inference_msg.header = self.latest_header

        for r in results:
            for box in r.boxes:

                det = InferenceResult()

                x1, y1, x2, y2 = map(int, box.xyxy[0])

                det.class_name = self.detection_model.names[int(box.cls[0])]
                det.confidence = float(box.conf[0])

                det.left = x1
                det.top = y1
                det.right = x2
                det.bottom = y2

                det.box_width = x2 - x1
                det.box_height = y2 - y1

                det.x = x1 + det.box_width / 2.0
                det.y = y1 + det.box_height / 2.0

                inference_msg.detections.append(det)

        annotated = results[0].plot()
        img_msg = bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
        img_msg.header = self.latest_header

        self.img_pub.publish(img_msg)
        self.yolo_pub.publish(inference_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    object_detection = ObjectDetection()
    rclpy.spin(object_detection)
    object_detection.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()