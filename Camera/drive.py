#!/usr/bin/env python

import sys
import rospy
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from std_msgs.msg import Float64
import time


class racer():
'''
    Class that handles the interaction with the car.
'''
    def __init__(self):
        '''
            Initializes the class. Creating one publisher that publishes speed messages and one publisher that publishes brake messages.
            Count is just a control varialbe during the testing of funcitonallity, will be removed later on.
        '''
        self.pub = rospy.Publisher('/vesc/low_level/ackermann_cmd_mux/input/navigation', AckermannDriveStamped , queue_size=1)
        self.brake_pub = rospy.Publisher('/vesc/commands/motor/brake', Float64, queue_size=1)
        self.count = 0

    def set_speed(self, linear, angular):
        '''
            Sets the current linear, and angular speed of the car.

            To set the speed, an AckermannDriveStamped message must be published to the navigation topic.

            Parameters:
            ---------------------
            linear (floar) : The linear speed of the car
            angular (float): The angular speed of the car.

            Returns: None
        '''
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = angular
        drive_msg.drive.speed = linear
        self.pub.publish(drive_msg)
    
    def brake(self, brake_pressure):
        '''
            Send the braking command to the car

            Paramters:
            --------------------
            brake_pressure (float) : The brake pressure to be applied. Min = ??, Max = 20000.

            Returns: None
        '''
        self.brake_pub.publish(brake_pressure)

        #drive_msg = AckermannDriveStamped()
        #drive_msg.drive.steering_angle = 0.0
        #drive_msg.drive.speed = 0.0
        #self.pub.publish(drive_msg)



####
#
#   Test program. 
#
####
if __name__=='__main__':
    rospy.init_node('best_node_ever', anonymous=True)
    print("node initialized")
    car = racer()
    print("Sampling rate set to 100Hz")
    rate = rospy.Rate(500)
    print("speed set to 1.0")
    speed = 3.0
    b_pressure = 15000

    while not rospy.is_shutdown():
        #rospy.loginfo(car.count)
        while (car.count < 1000):
           # print("speed increased")
           # speed = 0.0
            car.count = car.count + 1
            try:
                car.set_speed(speed, 0)
            except rospy.ROSInterruptException:
                pass

            print(car.count)
            rate.sleep()
            
        try:
            car.brake(b_pressure)
            #car.set_speed(0.0, 0.0)
        except rospy.ROSInterruptException:
            pass

        #)) Compensation on the servo to drive straight at different speeds.
        # At speed 1.0 servo = 0.005
        # At speed 2.0 servo = 0.0
        # At speed 3.0 servo slightly drifting right  
        # At speed 4.0 servo = 
        # At speed 5.0 servo = 



