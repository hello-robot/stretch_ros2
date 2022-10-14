#!/usr/bin/env python3

import cv2
import rclpy
from cv_bridge import CvBridge # Check if ros_numpy can be used instead
from sensor_msgs.msg import Image

if __name__ == '__main__':
    node = rclpy.create_node("webcam_node")
    rgb_topic_name = '/camera/color/image_raw' #'/camera/infra1/image_rect_raw'
    webcam_pub = node.create_publisher(Image, rgb_topic_name, queue_size=1)
    vid = cv2.VideoCapture(0)
    bridge = CvBridge()

    while(True):
        ret, frame = vid.read()
        cv2.imshow('frame', frame)
        image_message = bridge.cv2_to_imgmsg(frame, encoding="passthrough")
        webcam_pub.publish(image_message)
        # press 'q' to quit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # After the loop release the cap object
    vid.release()
    # Destroy all the windows
    cv2.destroyAllWindows()
