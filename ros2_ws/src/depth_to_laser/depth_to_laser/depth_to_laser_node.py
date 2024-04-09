import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from transformers import DPTImageProcessor, DPTForDepthEstimation
from PIL import Image as PILImage
import torch
import numpy as np
import cv2

class DepthToLaserNode(Node):
    def __init__(self):
        super().__init__('depth_to_laser_node')
        
        self.sub_camera = self.create_subscription(
            Image,
            'camera',
            self.publish_laser_transform,
            10)
        self.pub_depth = self.create_publisher(Image, "depth", 10)
        self.bridge = CvBridge()
        self.image_processor = DPTImageProcessor.from_pretrained("Intel/dpt-swinv2-tiny-256")
        self.model = DPTForDepthEstimation.from_pretrained("Intel/dpt-swinv2-tiny-256")

    def publish_laser_transform(self, msg):
        self.get_logger().info('Received Image')
        self.estimate_depth(msg)

    def estimate_depth(self, msg):
        # Convert from ROS Image to CV Image (which is a numpy array)
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

        # Resize the CV Image to the desired size (256x192)
        cv_image_resized = cv2.resize(cv_image, (256, 192), interpolation=cv2.INTER_AREA)

        # Convert the resized numpy array to PIL Image for processing
        image_transformed = PILImage.fromarray(cv_image_resized)
        inputs = self.image_processor(images=image_transformed, return_tensors="pt")

        with torch.no_grad():
            outputs = self.model(**inputs)
            predicted_depth = outputs.predicted_depth

        # Resizing the depth image to desired size (256x192)
        prediction = torch.nn.functional.interpolate(
            predicted_depth.unsqueeze(1),
            size=[192, 256],  # Adjusted size in the order of height, width
            mode="bicubic",
            align_corners=False,
        )

        # Convert to numpy array and normalize
        output = prediction.squeeze().cpu().numpy()
        formatted = (output * 255 / np.max(output)).astype(np.uint8)

        # Convert the numpy array back to a ROS Image
        depth_image_msg = self.bridge.cv2_to_imgmsg(formatted, encoding="mono8")
        self.pub_depth.publish(depth_image_msg)

        

def main(args=None):
    rclpy.init(args=args)
    node = DepthToLaserNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
