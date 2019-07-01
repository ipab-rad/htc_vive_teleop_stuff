import message_filters
from pr2_msgs.msg import AccelerometerState
import rospy

rospy.init_node('acc_filter', anonymous=True)

sub = message_filters.Subscriber('/accelerometer/l_gripper_motor', AccelerometerState)
cache = message_filters.Cache(sub, 100)


def sub(data):
    print([data])


rospy.spin()

