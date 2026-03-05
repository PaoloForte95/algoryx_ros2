#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from athena_msgs.msg import PlanningProblem
import agxROS2  
from sensor_msgs.msg import Image
from std_msgs.msg import Header
import numpy as np
import agxSDK
IMAGE_TOPIC = "/camera/camera/color/image_raw"
PLANNING_PROBLEM_TOPIC = "planning_problem"

class AGxImagePublisher ():
    def __init__(self, node: Node):
        self.node = node
        self.pub = node.create_publisher(Image, IMAGE_TOPIC, 10)
        self.sub = agxROS2.SubscriberSensorMsgsImage(IMAGE_TOPIC)

        self.msg_received_camera_image = agxROS2.SensorMsgsImage()

        self.timer = node.create_timer(0.1, self.timer_callback)
    
    def timer_callback(self):
        msg = Image()
        if self.sub.receiveMessage(self.msg_received_camera_image):
            msg.data = self.msg_received_camera_image.data
            msg.height = self.msg_received_camera_image.height
            msg.width = self.msg_received_camera_image.width
            msg.encoding = self.msg_received_camera_image.encoding
            msg.is_bigendian = self.msg_received_camera_image.is_bigendian
            msg.step = self.msg_received_camera_image.step
            
        self.pub.publish(msg)


    def publish_rgb(self, img: np.ndarray):
        img = np.asarray(img)
        if img.dtype != np.uint8:
            img = np.clip(img * 255.0, 0, 255).astype(np.uint8)

        if img.ndim == 2:
            img = np.repeat(img[:, :, None], 3, axis=2)
        elif img.ndim == 3 and img.shape[2] == 1:
            img = np.repeat(img, 3, axis=2)
        elif img.ndim == 3 and img.shape[2] >= 3:
            img = img[:, :, :3]
        else:
            raise ValueError(f"Unsupported image shape: {img.shape}")

        img = np.require(img, np.uint8, "C")
        h, w, _ = img.shape

        msg = Image()
        msg.header = Header()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        msg.height = h
        msg.width = w
        msg.encoding = "rgb8"
        msg.is_bigendian = 0
        msg.step = 3 * w
        msg.data = img.tobytes()

        self.pub.publish(msg)


class PlanningProblemSubscriber:
    def __init__(self, node: Node, topic: str):
        self.node = node

        # AGX publisher lives here now
        self.agx_pub = agxROS2.PublisherStdMsgsString(topic)

        self.sub = node.create_subscription(
            PlanningProblem,
            topic,
            self.listener_callback,
            1000
        )

    def listener_callback(self, msg):
        self.node.get_logger().info("Received PlanningProblem")

        relay_msg = agxROS2.StdMsgsString()
        relay_msg.data = "received planning problem"
        self.agx_pub.sendMessage(relay_msg)


class AgxBridgeNode(Node):
    def __init__(self, planning_topic: str, image_topic: str):
        super().__init__("agx_bridge_node")

        self.planning_sub = PlanningProblemSubscriber(self, planning_topic)
        self.image_pub = AGxImagePublisher(self)
        self.agx_planning_pub = agxROS2.PublisherStdMsgsString(planning_topic)

        self.get_logger().info("AgxBridgeNode ready")

        relay_msg = agxROS2.StdMsgsString()
        relay_msg.data = "received planning problem"
        self.agx_planning_pub.sendMessage(relay_msg)

    def publish_frame(self, rgb_img: np.ndarray):
        self.image_pub.publish_rgb(rgb_img)




# ---------------- MAIN ----------------
def main(args=None):
    rclpy.init(args=args)

    node = AgxBridgeNode(
        planning_topic=PLANNING_PROBLEM_TOPIC,
        image_topic=IMAGE_TOPIC
    )

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
