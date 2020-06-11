#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from or_msgs.msg import TrunkAngleMsg

def talker():
    rospy.init_node('talker', anonymous=True)
    # pub = rospy.Publisher('chatter', String, queue_size=10)
    pub = rospy.Publisher('chatterpy', TrunkAngleMsg, queue_size=10)
    rate = rospy.Rate(21)
    count = 0
    while not rospy.is_shutdown():
        # hello_str = "hello world %s" % rospy.get_time()
        # rospy.loginfo(hello_str)
        # pub.publish(hello_str)
        data = []
        data.append(count)
        # for i in range(int(count/10)):
        #     data.append(i)
        header = rospy.Header()
        header.stamp = rospy.Time.now()
        pub.publish(TrunkAngleMsg(header, data))
        count = count + 1
        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
