import rospy
from nav_msgs.msg import Odometry

quadrants = [False, False, False, True]
Flag = True

# Function that maps a position to a map quadrant.
def computeQuadrants(x,y):
    if x > 0 and y < 0:
        quadrants[0] = True
    elif x < 0 and y < 0 and quadrants[0]:
        quadrants[1] = True
    elif x < 0 and y > 0 and quadrants[1]:
        quadrants[2] = True
    elif x > 0 and y > 0 and quadrants[2]:
        quadrants[3] = True 

# Function that checks if the vehicle has passed the finish line
def justPassedFinishLine(x,y):
    if x > 0 and y < 0 and quadrants[3]:
        return True
    else:
        return False

# Print time on the terminal if lap is completed.
def printInfo():
    diff = rospy.get_time() - time_buffer
    global Flag
    if Flag:
        rospy.loginfo("pynode Lap time measurement started...")
        Flag = False
    else:
        rospy.loginfo("Lap time by python node: %fs", diff)

# Function that resets the data.
def resetData():
    global quadrants, time_buffer
    quadrants = [False, False, False, False]
    time_buffer = rospy.get_time()

# Callback for the incomming positon.
def callbackPosition(msg):
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    computeQuadrants(x,y)

    if justPassedFinishLine(x,y):
        print(y)
        printInfo()
        resetData()




if __name__ == "__main__":
    # Initialize the ROS node
    rospy.init_node('pyvehicle_timer')
    rospy.loginfo("pyVehicle timer started.")

    # Get subscribe topic from parameter server
    subscribe_topic_position = rospy.get_param("vehicle/position_topic")
    rospy.loginfo("pyVehicle timer subscribes to: %s", subscribe_topic_position)

    #intialize time buffer
    time_buffer = rospy.get_time()

    # Connect subscriber and to the respective topics and callback function
    subscriber_position_data = rospy.Subscriber(subscribe_topic_position, Odometry, callbackPosition, queue_size=10)

    # Enter a loop to keep the node running while looking for messages on the subscribed topic
    rospy.loginfo("pyVehicle timer is running...")
    rospy.spin()
