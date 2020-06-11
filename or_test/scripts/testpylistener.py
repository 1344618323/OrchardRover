#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from or_msgs.msg import TrunkAngleMsg


# def callback(data):
#     rospy.loginfo(rospy.get_caller_id()+" I heard %s", data.data)

# def listener():
#     rospy.init_node('listener', anonymous=True)
#     rospy.Subscriber('chatter', String, callback)
#     rospy.spin()


def callback(data):
    print data.angle


def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('chatter', TrunkAngleMsg, callback, queue_size=1)
    rospy.spin()


if __name__ == '__main__':
    listener()
