#!/usr/bin/env python
import roslib
roslib.load_manifest('camera')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

  def __init__(self):
    #self.image_pub = rospy.Publisher("image_topic_2",Image, queue_size=1)

    #cv2.namedWindow("Image window", 1)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/color/image_raw",Image,self.callback)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError, e:
        print e

    (cols,rows, channels) = cv_image.shape
    if cols > 60 and rows > 60 :
      cv2.circle(cv_image, (50,50), 10, 255)

    grey = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    grey = cv2.blur(grey, (7, 7))
    edges = cv2.Canny(grey, 15.0, 30.0)
    print(edges)
    #cv2.imshow("Image window", edges)
    #cv2.waitKey(3)
    
    try:
        print('TRYING TO SAVE THIS IMAGE')
        cv2.imwrite('/home/crisp-tron/crisp_img/camera_image.jpeg', cv_image)
        #cv2.imwrite('/home/crisp-tron/crisp_img', self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except:
        print("YOU KNOW NOTHING!")

    #try:   
    #  self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    #except CvBridgeError, e:
    #  print e

def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
