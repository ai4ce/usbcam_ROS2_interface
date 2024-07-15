from usbcam_interface_msg.srv import TakeImage

from sensor_msgs.msg import Joy

from queue import Queue
import os

import cv2
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node



class ImageClient(Node):

    def __init__(self):
        super().__init__('image_client') # type: ignore


        ############################ Miscanellous Setup #######################################
        self.cvbridge = CvBridge() # for converting ROS images to OpenCV images

        self._debounce_setup() # for debouncing the capture button
        self.shutter = False # when this is true, the client will issue a request to the server to capture images

        ############################ Launch Parameters ########################################
        # parameter handling
        self.declare_parameter(name = 'save_folder', value = '/home/irving/Desktop')
        self.save_folder = self.get_parameter('save_folder').get_parameter_value().string_value

        ############################ Client Setup #############################################
        # image client
        self.image_cli = self.create_client(
            srv_type=TakeImage, 
            srv_name='/usbcam_capture/get_image')
        while not self.image_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('RGB service not available, waiting again...')
        
        
        self.image_req = TakeImage.Request()

        ############################ Subscriber Setup #########################################
        # subscribe to joy topic to read joystick button press
        self.joy_sub = self.create_subscription(
            msg_type=Joy, 
            topic='/joy', 
            callback=self.joy_callback, 
            qos_profile=10)

    def joy_callback(self, msg):
        
        old_value = self.debounce_buffer.get() # pop the oldest read
        self.debounce_buffer.put(msg.buttons[8]) # push the newest read
        if old_value == 0 and msg.buttons[8] == 1: 
            self.shutter = True # rising edge detected

    def postprocess(self, img):
        '''
        img: sensor_msgs.msg.Image
        modality: str
        Use cv_bridge to convert ROS image to OpenCV image and save it to disk
        '''
        # this is in RGB
        encoded_img = self.cvbridge.imgmsg_to_cv2(img_msg=img, 
                                                  desired_encoding='passthrough')
        cv2.imwrite(os.path.join(self.save_folder, f'{img.header.stamp.sec}.png'), cv2.cvtColor(encoded_img, cv2.COLOR_RGB2BGR))

        # color the log message
        color_start = '\033[94m'
        color_reset = '\033[0m'

        self.get_logger().info(f'{color_start} image saved{color_reset}')
        return encoded_img

    def _debounce_setup(self):
        '''
        As in any embedded system, we need to debounce the capture button.
        While we human think we press the button once, the computer actually consider the button pressed all the time during the duration of the press,
        because the polling rate is much faster than the human reaction time. 
        
        This function sets up a buffer to store the last value of the button press so that we can detect the rising edge.
        '''

        self.debounce_buffer = Queue(maxsize=1)
        self.debounce_buffer.put(0) # when nothing is pressed, the value is 0

def main(args=None):
    rclpy.init(args=args)

    client = ImageClient()
    while rclpy.ok():
        if client.shutter: # shutter down

            # send request to server to capture images
            image_future = client.image_cli.call_async(client.image_cli)

            # immediately shutter up to debounce, so we don't caputre multiple images
            client.shutter = False
            
            client.get_logger().info('Request to capture rgb image sent...')
            
            # wait for the server to capture images
            rclpy.spin_until_future_complete(client, image_future)

            # get the images from the server
            client.get_logger().info('Images Acquired...')
            image_response = image_future.result()

            # postprocess the images
            client.postprocess(image_response.frame)

        rclpy.spin_once(client)

    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()