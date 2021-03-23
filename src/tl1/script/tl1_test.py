#!/usr/bin/env python


from openai_ros import robot_gazebo_env


from sensor_msgs.msg import JointState
from gazebo_msgs.msg import LinkStates
from std_msgs.msg import Float64

import rospy

rospy.init_node('tl1_test', anonymous=True, log_level=rospy.FATAL)

_right_pub = rospy.Publisher('/tl1/leg_right_controller/command', Float64, queue_size=1)
_left_pub = rospy.Publisher('/tl1/leg_right_controller/command', Float64, queue_size=1)


i = 0
j = 0

while True:
    joint_value_ll = Float64()
    joint_value_ll.data = i

    joint_value_rl = Float64()
    joint_value_rl.data = j
    
    _left_pub.publish(joint_value_ll)
    _right_pub.publish(joint_value_rl)
    i += 1
    j += 1
    print(i)
    print(j)
    print()