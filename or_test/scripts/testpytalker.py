#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from or_msgs.msg import TrunkAngleMsg

def talker():
    rospy.init_node('talker', anonymous=True)
    # pub = rospy.Publisher('chatter', String, queue_size=10)
    pub = rospy.Publisher('chatter', TrunkAngleMsg, queue_size=10)
    rate = rospy.Rate(10)
    count=0
    while not rospy.is_shutdown():
        # hello_str = "hello world %s" % rospy.get_time()
        # rospy.loginfo(hello_str)
        # pub.publish(hello_str)
        data=[]
        for i in range(int(count/10)):
            data.append(i)
        pub.publish(TrunkAngleMsg(None,data))
        count=count+1
        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
