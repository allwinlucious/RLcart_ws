import rospy
from nav_msgs.msg import Odometry


# Callback for the incomming positon.
def callbackPosition(msg):
    Vx = msg.twist.twist.linear.x
    Vy = msg.twist.twist.linear.y
    if abs(Vx) <0.001 and abs(Vy) < 0.001 :
        rospy.loginfo("Crash detected")
    

# Initialize the ROS node
rospy.init_node('vehicle_crash_checker')
rospy.loginfo("vehicle crash checker started.")

# Get subscribe topic from parameter server
subscribe_topic_position = rospy.get_param("vehicle/position_topic")
rospy.loginfo("Vehicle crash check subscribes to: %s", subscribe_topic_position)
rospy.loginfo("Vehicle crash check is running...")	
while not rospy.is_shutdown():

    # Connect subscriber and to the respective topics and callback function
    subscriber_position_data = rospy.Subscriber(subscribe_topic_position, Odometry, callbackPosition)

    rospy.sleep(100)
