import numpy as np
import rospy
from flatland_msgs.srv import DeleteModel, SpawnModel
from geometry_msgs.msg import Pose2D, Twist
from gym import Env
from gym.spaces import Box
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from stable_baselines3.common.env_checker import check_env
from std_srvs.srv import Empty
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import ProgressBarCallback


def interval_mapping(x, from_min, from_max, to_min, to_max):
    # map values from [from_min, from_max] to [to_min, to_max]
    if type(x) is not list :x = [x]
    from_range = from_max - from_min
    to_range = to_max - to_min
    scaled = (x - from_min*np.ones(len(x))) / from_range
    return to_min + (scaled * to_range)


class racecar():
   
    def __init__(self,debug=False):
        

        self.new_action = Twist() # linear velocity, steering angle
        self.observation =  np.zeros(5, dtype=np.float32)
        self.state = np.zeros(5, dtype=np.float32) # state is [x,y,vx,vy,track_time]
        self.reward = 0
        self.done = False
        self.info = {}

        #time
        self.start_time = rospy.get_time()
        self.track_time = 0
        self.laps = 0

        self.quadrants = [False,False,False,False]
        self.debug = debug

        # initialize subscribers and publishers
        subscribe_topic_sensors = rospy.get_param("vehicle/sensor_topic")
        publish_topic_actuators = rospy.get_param("vehicle/actuator_topic")
        subscribe_topic_position = rospy.get_param("vehicle/position_topic")

        self.publisher_actions = rospy.Publisher(publish_topic_actuators, Twist,queue_size=1)
        rospy.Subscriber(subscribe_topic_sensors, LaserScan, self.getObservation)
        rospy.Subscriber(subscribe_topic_position, Odometry, self.getState)


        
    def spawnModel(self):
        rospy.wait_for_service('spawn_model')
        try:
            spawn_model = rospy.ServiceProxy('spawn_model', SpawnModel)
            pose = Pose2D()
            pose.x = 2
            pose.y = 6
            pose.theta = -1.57
            rsp2 = spawn_model('racing_cart.yaml','racing_cart','',pose)
            if not rsp2.success:
                rospy.loginfo("Failed to spawn model")
        except rospy.ServiceException as e:
            print("Service call for spawn_model failed: %s"%e)
    
    def delete(self):
        rospy.wait_for_service('delete_model')
        try:
            delete_model = rospy.ServiceProxy('delete_model', DeleteModel)
            rsp = delete_model('racing_cart')
            if not rsp.success:
                rospy.loginfo("Failed to delete model")
        except rospy.ServiceException as e:
            print("Service call for delete_model failed: %s"%e)

    def getObservation(self, msg):
        self.observation = np.array(interval_mapping(msg.ranges,0,40,-1,1), dtype=np.float32)
    
    def updatetracktime(self):
        self.track_time = rospy.get_time() - self.start_time
    
    def getState(self,msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        Vx = msg.twist.twist.linear.x
        Vy = msg.twist.twist.linear.y
        self.updatetracktime()
        self.state = (x,y,Vx,Vy,self.track_time)
    

    def act(self,action):
        self.new_action.linear.x = interval_mapping(action[0], -1, 1, -5, 30)[0]
        self.new_action.angular.z = interval_mapping(action[1], -1, 1, -0.7, 0.7)[0]
        self.publisher_actions.publish(self.new_action)
    
    def computeReward(self):
        if self.isCrashed():
            self.info = {}
            self.done = True
            self.reward += -100
            if self.debug == True:
                rospy.loginfo('Crashed!, reward :%d',self.reward)
        else:
            self.reward += np.sqrt(np.square(self.state[2])+np.square(self.state[3]))/1000
        if self.isLapComplete():
            self.reward += 1000
            if self.debug == True:
                rospy.loginfo('Lap Complete!, reward :%d',self.reward)
            self.quadrants = [False,False,False,False]
            self.laps += 1
    
    def isCrashed(self):
        if self.state[4] < 1: #allowing cart to accelerate
            if self.debug == True:
                rospy.loginfo('Cart started :%d',self.state[4])
            return False
        else: 
            if abs(self.state[2]) <0.9 and abs(self.state[3]) < 0.9 :
                return True
            else:
                return False
    def computequadrants(self,x,y):
        if x > 0 and y < 0:
            self.quadrants[0] = True
            
        elif x < 0 and y < 0 and self.quadrants[0]:
            self.quadrants[1] = True
            
        elif x < 0 and y > 0 and self.quadrants[1]:
            self.quadrants[2] = True
            
        elif x > 0 and y > 0 and self.quadrants[2]:
            self.quadrants[3] = True 

    def justPassedFinishLine(self):
        self.computequadrants(self.state[0],self.state[1])
        if self.state[0] > 0 and self.state[1] < 0 and self.quadrants[1]:
            return True
        else:
            return False

    def isLapComplete(self):
        if self.justPassedFinishLine() and all(self.quadrants):
            return True
        else:
            return False

class racetrack_env(Env):
    def __init__(self):

        #define action and observation space
        self.observation_space = Box(low=np.array([0,0,0,0,0]),high=np.array([40,40,40,40,40]), shape=(5,), dtype=np.float32)
        self.action_space = Box(low=np.array([-1,-1]), high=np.array([1,1]),shape=(2,), dtype=np.float32)
        
        self.agent = racecar()
        
    def step(self, action):
        #self.resume()
        self.agent.act(action)
        #self.pause()
        self.agent.computeReward()
        return self.agent.observation, self.agent.reward, self.agent.done, self.agent.info

    def render(self):
        #already done by the simulator
        pass

    def reset(self):
        self.agent.delete()
        del self.agent
        self.agent = racecar()
        self.agent.spawnModel()
        return self.agent.observation

rospy.init_node('vehicle_controller')
rospy.loginfo("Vehicle controller started.")

n_episodes= 10
env = racetrack_env()
check_env(env)
rate = rospy.Rate(50) # 50hz
model= PPO.load("PPO_racing_cart3",env=env)    
for episode in range(n_episodes):
    obs = env.reset()
    done = False
    while  not rospy.is_shutdown():
        action, _ = model.predict(obs, deterministic=True)
        action = action.reshape(2,)
        obs, reward, done, info = env.step(action)
        if done == True:
            break
        rate.sleep()
rospy.spin()