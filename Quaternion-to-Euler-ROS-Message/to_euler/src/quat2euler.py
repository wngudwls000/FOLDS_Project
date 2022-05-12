#!/usr/bin/env python
# The following code converts quaternion to euler angles in units of degrees
# TODO: write custom message of type Vector3
# TODO: correct for magnetic declination in the param config file
# Andinet Hunde
# September 2019, cuicar
import rospy
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3
from tf.transformations import euler_from_quaternion

pub = rospy.Publisher  ('/myEuler',Vector3,queue_size=10)

def euler_Callback (data):
    vec = Vector3()
    q = data.pose.pose.orientation
    quat = [q.x, q.y, q.z, q.w]
    (roll, pitch, yaw) = euler_from_quaternion (quat)
    vec.x = math.degrees(roll)
    vec.y = math.degrees(pitch)
    vec.z = math.degrees(yaw)
    pub.publish(vec)

def run():
    rospy.init_node('quat2euler',anonymous=True)
    sub = rospy.Subscriber ('/odom', Odometry, euler_Callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass
