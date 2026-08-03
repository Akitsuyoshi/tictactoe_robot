import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os


class CameraSubscriber(Node):

    def __init__(self):
        super().__init__('camera_dataset_saver')

        self.declare_parameter("split", "train")
        split = self.get_parameter("split").value

        topic_n = '/camera1/image_raw'
        if split == "test":
            topic_n = '/camera/D435/color/image_raw'

        self.subscription = self.create_subscription(
            Image,
            topic_n,
            self.listener_callback,
            10
        )
        self.br = CvBridge()


        self.save_dir = f"/home/user/dataset/images/{split}"
        os.makedirs(self.save_dir, exist_ok=True)
        images = sorted(
            f for f in os.listdir(self.save_dir)
            if f.endswith(".jpg")
        )
        self.image_id = len(images)
        
        self.saved = False


    def listener_callback(self,msg):
        if self.saved:
            return
        frame = self.br.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        filename = os.path.join(self.save_dir, f"image_{self.image_id:05d}.jpg")

        h, w = frame.shape[:2]
        self.get_logger().info(f"Image size: {w} x {h}")

        # Save the image
        cv2.imwrite(filename, frame)

        self.get_logger().info(f"Saved {filename}")

        self.saved = True




def main(args=None):
    rclpy.init(args=args)
    node = CameraSubscriber()
    while rclpy.ok() and not node.saved:
        rclpy.spin_once(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()