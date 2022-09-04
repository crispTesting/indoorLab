#############################################################################
#                                                                           #
#   Next steps:                                                             #
#                                                                           #                                                                                                                                                                                                               #
#   Properly implement the calculation of heading angle.                    #
#                                                                           #
#   Implement storage of global variables to store last know location       #
#   of the lane.                                                            #
#   -   Explore how to deal with global variabes using ROS.                 #
#                                                                           #
#   Measure the width of the lane for different scenarious and store them   #
#   in a table.                                                             #
#                                                                           # 
# -   Can be stored in E.G utilis, or maybe better in a speartae file       #
#     as an ENUM.                                                           #
#                                                                           #
#   Find the lateral error.                                                 #
#                                                                           #                                                                                                                                    #                                                                                                                   #
#                                                                           #
#   LAST THING TO DO:                                                       #                                                                                                 
#   Write a comprehensive documentation of the CriSp project                #                                                                                 
#   -   What has been done, thoughts of future works, improvements          #
#       on existing solutions.                                              #
#                                                                           #
#   -   Thougts around paths not taken, motivation of why this              #
#       way was chosen. - Software, hardware, track design.                 #
#                                                                           #
#   -   List of materials - On the car - for the track,                     #
#       left in spare parts box.                                            #
#                                                                           #
#   REMEMBER:                                                               #
#                                                                           #
#   Must upload latest scripts to the f1tenth car, and make sure            #
#   everything is working correctly.                                        #
#                                                                           #
#                                                                           #
#                                                                           #
#############################################################################


#############################################################################
#                         IMAGE PROCESSING FOR CRISP                        #
#############################################################################  
import cv2
import numpy as np
from matplotlib import pyplot as plt
import time
from utilities import *
import math

#############################################################################
#                         READING THE IMAGE                                 #
#############################################################################  
# Absolute path to image
path = r'/home/robotlab/crisp_git/crisp/test_images/high_mount/49.jpeg'

# Loading the image from path
img = cv2.imread(path)

#############################################################################v
#                         PROCESSING IMAGE                                  #
#############################################################################  
resize_dimensions = (427, 240)
# Kernel used for dilation and erosion
kernel = np.ones((3,3), np.uint8)

# Gets the processed image
img_processed = process_image(img, resize_dimensions, kernel)
# Resize
img_org_resized = resize(img, resize_dimensions)
# Copy of original, used for drawing on.
img_org_resized_2 = resize(img, resize_dimensions)

#############################################################################
#                         CALCULATING HORIZONTALS                           #
#############################################################################  
height = img_processed.shape[0]
bottom_horizontal = int(height*0.6)    # 0.8 # height-1
mid_horizontal = int( height * 0.4)  # 0.4 # 0.9
mid_horizontal_2 = int(height * 0.3)
top_horizontal = int (height * 0.2)   # 0.2 # 0.8

#############################################################################
#                        DYNAMIC CALCULATING HORIZONTALS                    #
#############################################################################
height = img_processed.shape[0]
d_horizontals = get_horizontals(img_processed, height)
#print(d_horizontals)
found_lane = find_lane(d_horizontals,img_processed)
#print(found_lane)
img_copy = img_org_resized.copy()
hor = []

for i in range(len(found_lane)):
    segment = []
    
    if found_lane[i][3] > 50: 
        segment.append(d_horizontals[i])
        segment.append(found_lane[i][0])
        segment.append(found_lane[i][1])
        hor.append(segment)

#print(hor)

draw_horizontals_dynamic(img_copy, hor)


#############################################################################
#                         GETING LANE EDGES                                 #
#############################################################################  
h1_l, h1_r, h1_l_e, h1_r_e = scan_for_lane_mid(bottom_horizontal, img_processed)
h2_l, h2_r, h2_l_e, h2_r_e = scan_for_lane_mid(mid_horizontal, img_processed)
h2_l_2, h2_r_2, h2_l_e_2, h2_r_e_2 = scan_for_lane_mid(mid_horizontal_2, img_processed)
h3_l, h3_r, h3_l_e, h3_r_e = scan_for_lane_mid(top_horizontal, img_processed)


#############################################################################
#               CALCULATING MIDDLEPOINTS FOR EACH HORIZONTAL                #
#############################################################################  
m1 = int( (h1_r - h1_l) / 2) +h1_l
m2 = int( (h2_r - h2_l) / 2) + h2_l
m2_2 = int( ((h2_r_2 - h2_l_2) / 2) ) + h2_l_2
m3 = int( (h3_r - h3_l) / 2) + h3_l


# List of horizonbtals and left and right lane pixels
#horizontals = [[bottom_horizontal, h1_l, h1_r], [mid_horizontal, h2_l, h2_r], [mid_horizontal_2, h2_l_2, h2_r_2], [top_horizontal, h3_l, h3_r]]
#middlepoints = [m1, m2]

I_mid = img_processed.shape[1] / 2


#############################################################################
#     CALCULATING THE COVERSITIONAL CONSTANT BETWEEN PIXELS AND CM          #
#############################################################################  
h1_c = get_pw_to_cm_conversion_constant(h1_r, h1_l)
h2_c = get_pw_to_cm_conversion_constant(h2_r, h2_l)
h2_2_c = get_pw_to_cm_conversion_constant(h2_r_2, h2_l_2)
h3_c = get_pw_to_cm_conversion_constant(h3_r, h3_l)

#############################################################################
#         CALCULATING LATERAL ERROR, HEADING ERROR AND CURVATURE            #
#############################################################################  
lateral_error = get_lateral_error(I_mid, m2, h2_c)
theta_actal = (h2_l- h2_l_e ) / h2_c
h_error = get_heading_error(theta_actal)
curvature = get_curvature(h1_l, h2_l, h2_l_2, h3_l, bottom_horizontal, mid_horizontal, mid_horizontal_2, top_horizontal)

#############################################################################
#                         PRINTING DATA TO CONSOLE                          #
#############################################################################
'''
print()
print("*************************************************************************************************")
print("*\t\t\t\t\tLANE DETECTION DATA\t\t\t\t\t*")
print("*\t\t\t\t\t\t\t\t\t\t\t\t*")                                                                         
print("*\tLEFT:\tLEFT END:\tRIGHT:\tRIGHT END:\tY:\tMid\tpw to cm\t\t*")
print(f"*First:\t{h1_l}\t{h1_l_e}\t\t{h1_r}\t{h1_r_e}\t\t{bottom_horizontal}\t{m1}\t{h1_c}\t*")
print(f"*Second:{h2_l}\t{h2_l_e}\t\t{h2_r}\t{h2_r_e}\t\t{mid_horizontal}\t{m2}\t{h2_c}\t*")
print(f"*Third:\t{h2_l_2}\t{h2_l_e_2}\t\t{h2_r_2}\t{h2_r_e_2}\t\t{mid_horizontal_2}\t{m2_2}\t{h2_c}\t*")
print(f"*Fourth\t{h3_l}\t{h3_l_e}\t\t{h3_r}\t{h3_r_e}\t\t{top_horizontal}\t{m3}\t{h3_c}\t\t\t*")
print("*************************************************************************************************")
print("\n")

print("*************************************************************************************************")
print("*\t\t\t\t\tERROR CALCULATIONS\t\t\t\t\t*")
print("*\t\t\t\t\t\t\t\t\t\t\t\t*")       
print("*Lateral error (cm)\t\tTheta actual\tHeading error\t\tCurvature\t\t\t*")
print(f"{lateral_error}\t{theta_actal}\t\t{h_error}\t{curvature}\t\t*")
print("*************************************************************************************************")
print()
'''

#print_lane_data(hor)   

#############################################################################
#                         DRAWING ON IMAGES                                 #
#############################################################################  
# Drawing the horizontals
#img_draw = draw_horizontals(img_org_resized, horizontals)
# Drawing a heading vector on the image.
#draw_heading(img_draw, horizontals, middlepoints)


#############################################################################
#                         DISPLAYING IMAGES                                 #
#############################################################################  
# Making the binary image have 3_channels - needed for displaying
img_processed_3_channel = cv2.cvtColor(img_processed, cv2.COLOR_GRAY2BGR)
# Stacking the images in a stack before displaying
img_stack = np.concatenate( (img_processed_3_channel, img_org_resized), axis=0)

# Using matplotlib to display the images in a nice way.
fig, axs = plt.subplots(3)
fig.suptitle('Image processing')
axs[0].imshow(img_org_resized_2)
axs[1].imshow(img_processed, cmap='gray')
axs[2].imshow(img_copy)
#axs[2].imshow(img_org_resized)
axs[0].set_title("Original image")
axs[1].set_title("Triangular binarization")
axs[2].set_title("Original image, with horizontals and heading")
fig.tight_layout()
plt.show()

cv2.waitKey(0)
cv2.destroyAllWindows()