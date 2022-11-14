import rospy
from geometry_msgs.msg import Pose2D
from flatland_msgs.srv import DeleteModel, SpawnModel


def reset():
    #delete model
    rospy.wait_for_service('delete_model')
    try:
        rospy.loginfo("Deleting model")
        delete_model = rospy.ServiceProxy('delete_model', DeleteModel)
        rsp = delete_model('racing_cart')
        print(rsp.success,rsp.message)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
    #spawn model
        rospy.wait_for_service('spawn_model')
    try:
        rospy.loginfo("Spawning model")
        spawn_model = rospy.ServiceProxy('spawn_model', SpawnModel)
        pose = Pose2D()
        pose.x = 2
        pose.y = 6
        pose.theta = -1.57
        rsp2 = spawn_model('racing_cart.yaml','racing_cart','',pose)
        print(rsp2.success,rsp2.message)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

rospy.init_node('reset_node')
rospy.loginfo("reset node started.")
reset()