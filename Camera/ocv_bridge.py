#!/usr/bin/env python
import roslib
roslib.load_manifest('camera')
import sys
import rospy
import cv2
import numpy as np
import utilities as u
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
	
    # Crop dimensions for faster image processing
    crop_dimensions = (427, 240)
    # Kernel used for dilation and erosion
    kernel = np.ones((3,3), np.uint8)

    img_org_resized = u.resize(cv_image, crop_dimensions)
    img_processed = u.process_image(cv_image, crop_dimensions, kernel)
    
    # Calculating horizontals
    height = img_processed.shape[0]
    first_horizontal = int(height * 0.8) 
    second_horizontal = int( height * 0.4)
    third_horizontal = int (height * 0.2)
    
    # Getting left and right lanes
    h1_l, h1_r = u.scan_for_lane_mid(first_horizontal, img_processed)
    h2_l, h2_r = u.scan_for_lane_mid(second_horizontal, img_processed)
    h3_l, h3_r = u.scan_for_lane_mid(third_horizontal, img_processed)
    
    # Calculating middle points for all horizontals
    m1 = int( (h1_r - h1_l) / 2) + h1_l
    m2 = int( (h2_r - h2_l) / 2) + h2_l
    m3 = int( (h3_r - h3_l) / 2) + h3_l
   
    horizontals = [[first_horizontal, h1_l, h1_r], [second_horizontal, h2_l, h2_r], [third_horizontal, h3_l, h3_r]]
    _ = u.draw_horizontals(img_org_resized, horizontals)
    cv2.imshow("Original",img_org_resized)
    cv2.imshow("Thresholded", img_processed)
    cv2.waitKey(1)
    
    #try:
    #    print('TRYING TO SAVE THIS IMAGE')
    #    cv2.imwrite('/home/crisp-tron/crisp_img/high_mount/49.jpeg', cv_image)
    #    #cv2.imwrite('/home/crisp-tron/crisp_img', self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    #except:
    #    print("YOU KNOW NOTHING!")

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
