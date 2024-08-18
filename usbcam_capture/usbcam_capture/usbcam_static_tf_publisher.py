from geometry_msgs.msg import TransformStamped

import rclpy
from rclpy.node import Node

from rclpy.time import Time
from rclpy.duration import Duration

from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class USBCamStaticTFPublisher(Node):
    """
    Broadcast transforms that never change.
    """

    def __init__(self):
        super().__init__('USBCam_static_tf_publisher') # type: ignore
        self.d405_center2left = 9.0/1000
        self.d405_height = 23.0/1000
        self.mount_translation_y = 69.66/1000
        self.mount_translation_z = 41.1/1000
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)

        
        # ############################ TF Setup ########################################
        self.link_name = 'link_usbcam'
        # Publish static transforms once at startup
        self.make_transforms()

        # buffer to hold the transform in a cache
        self.tf_buffer = Buffer()

        # listener. Important to spin a thread, otherwise the listen will block and no TF can be updated
        self.tf_listener = TransformListener(buffer=self.tf_buffer, node=self, spin_thread=True)

        self.pose_publisher = self.create_publisher(
        msg_type=TransformStamped, 
        topic='/usbcam_capture/usbcam_pose', 
        qos_profile=10)

        self.create_timer(0.5, self.publish_pose)
    
    def publish_pose(self):
        try:
            t = self.tf_buffer.lookup_transform('link_base', self.link_name, Time(), timeout=Duration(seconds=2))
            self.pose_publisher.publish(t)
        except Exception as e:
            self.get_logger().info(f"Failed to publish pose: {e}")

    def make_transforms(self):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'link_eef'
        t.child_frame_id = self.link_name

        t.transform.translation.x = float(-self.d405_center2left)
        t.transform.translation.y = float(-self.mount_translation_y)
        t.transform.translation.z = float(self.mount_translation_z+self.d405_height)

        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0


        self.tf_static_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = USBCamStaticTFPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()