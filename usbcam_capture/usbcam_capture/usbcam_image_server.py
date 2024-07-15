import cv2
from cv_bridge import CvBridge

from threading import Thread, Lock

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from usbcam_interface_msg.srv import TakeImage
from sensor_msgs.msg import Image

class Camera2D:
    """A mult-threaded wrapper around the cv2.VideoCapture"""
    def __init__(self, USB_id, imgh, imgw):
        self.imgh = imgh
        self.imgw = imgw
        
        self.started = False
        self.read_lock = Lock()

        """Connect to the camera using cv2 streamer."""
        self.stream = cv2.VideoCapture(USB_id)
        self.stream.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if self.stream is None or not self.stream.isOpened():
            print("Warning: unable to open video source: ", USB_id)
        (self.grabbed, self.frame) = self.stream.read()

    def start(self):
        if self.started:
            print("already started!!")
            return None
        self.started = True
        self.thread = Thread(target=self.update, args=())
        self.thread.start()
        return self
    
    def update(self):
        while self.started:
            (grabbed, frame) = self.stream.read()
            self.read_lock.acquire()
            self.grabbed, self.frame = grabbed, frame
            self.read_lock.release()

    def read(self):
        self.read_lock.acquire()
        frame = self.frame.copy()
        self.read_lock.release()
        return frame

    def stop(self):
        self.started = False
        self.thread.join()

class USBCamImageServer(Node):

    def __init__(self):
        super().__init__('usbcam_image_server') # type: ignore

        ############################ Miscanellous Setup #######################################
        multithread_group = ReentrantCallbackGroup()
        self.cvbridge = CvBridge() # for converting ROS images to OpenCV images
        
        # the actual camera that captures the images
        self.camera2d = Camera2D(USB_id=-1, 
                                 imgh=640, 
                                 imgw=480)
        self.camera2d.start()

        ############################ Publisher Setup ###########################################
        # image publisher
        self.image_publisher = self.create_publisher(
            msg_type=Image, 
            topic='/usbcam_capture/rgb_image', 
            qos_profile=10)
        image_timer_period = 0.05  # in seconds. equal to 20 Hz
        self.image_timer = self.create_timer(
            timer_period_sec=image_timer_period, 
            callback=self.image_timer_callback,
            callback_group=multithread_group)
        
        ############################ Service Setup ############################################$
        self.image_service = self.create_service(
            srv_type=TakeImage, 
            srv_name='/usbcam_capture/get_image', 
            callback=self.usbcam_srv_callback)

        
    def image_timer_callback(self):
        original_image = self.camera2d.read()
        # self.get_logger().info(f'image type: {type(original_image)}')
        ros_image = self.cvbridge.cv2_to_imgmsg(original_image, encoding='bgr8')
        self.image_publisher.publish(ros_image)
    
    def usbcam_srv_callback(self, request, response):
        original_image = self.camera2d.read()
        ros_image = self.cvbridge.cv2_to_imgmsg(original_image, encoding='bgr8')
        response.frame = ros_image
        return response
        



def main(args=None):
    rclpy.init(args=args)

    server = USBCamImageServer()

    executor = MultiThreadedExecutor()
    executor.add_node(server)

    executor.spin()

    server.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
