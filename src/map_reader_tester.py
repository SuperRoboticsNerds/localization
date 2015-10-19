import rospy
from std_msgs.msg import Bool

def talker():
    pub = rospy.Publisher('/map_reader/query', Bool, queue_size=10)
    rospy.init_node('map_reader_tester', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        message = Bool()
        pub.publish(message)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass