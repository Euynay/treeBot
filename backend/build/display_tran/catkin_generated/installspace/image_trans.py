#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import base64
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2

from cv_bridge import CvBridge, CvBridgeError
import json
from websocket import create_connection

def image_callback(image_message):
    print("Received an image!")  # 输出接收到图像的确认信息
    bridge = CvBridge()
    try:
        # 将ROS图像消息转换为OpenCV图像
        cv_image = bridge.imgmsg_to_cv2(image_message, "bgr8")
        print("Image conversion successful!")  # 转换成功的确认信息

        # 可选：显示图像进行验证
        cv2.imshow("Converted Image", cv_image)
        cv2.waitKey(1)  # 显示窗口等待的时间，这里设置为1毫秒
        
    except CvBridgeError as e:
        print(e)
        return

    # 将图像转换为JPEG格式
    result, encoded_image = cv2.imencode('.jpg', cv_image)
    if result:
        # 将JPEG图像转换为Base64字符串
        encoded_data = base64.b64encode(encoded_image.tobytes())
        # 准备发布的数据
        data_to_publish = json.dumps({'image': encoded_data.decode('utf-8')})
        # 发布数据到一个新的话题
        pub.publish(data_to_publish)

def main():
    global pub
    rospy.init_node('image_to_web')
    pub = rospy.Publisher('/web_image', String, queue_size=10)
    image_sub = rospy.Subscriber('/camera/image_raw', Image, image_callback)
    rospy.spin()

if __name__ == '__main__':
    main()

