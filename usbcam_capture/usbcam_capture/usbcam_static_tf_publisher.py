from geometry_msgs.msg import TransformStamped

import rclpy
from rclpy.node import Node

from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

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

        # Publish static transforms once at startup
        self.make_transforms()

    def make_transforms(self):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'link_eef'
        t.child_frame_id = 'link_camera'

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