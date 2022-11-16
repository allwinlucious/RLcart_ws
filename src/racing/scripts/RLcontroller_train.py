from gym import Env
from gym.spaces import Box
import rospy
from sensor_msgs.msg import LaserScan 
from geometry_msgs.msg import Twist, Pose2D
from flatland_msgs.srv import DeleteModel, SpawnModel
from nav_msgs.msg import Odometry
from stable_baselines3.common.env_checker import check_env
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import ProgressBarCallback
from stable_baselines3.common.callbacks import EvalCallback
import numpy as np



def isCrashed(state):
    if state[4] < 1: #allowing cart to accelerate
        #print('Cart started..',state[4])
        return False
    else: 
        if abs(state[2]) <0.9 and abs(state[3]) < 0.9 :
            return True
        else:
            #print(abs(state[2]) , abs(state[3]))
            return False
def isCrashed2(obs):
    #if any obs less then crashed
    if any(obs < 0.005):
        return True
def interval_mapping(x, from_min, from_max, to_min, to_max):
    # map values from [from_min, from_max] to [to_min, to_max]
    # image: input array
    from_range = from_max - from_min
    to_range = to_max - to_min
    scaled = np.array((x - from_min) / np.float32(from_range), dtype=np.float32)
    return to_min + (scaled * to_range)

class racing_cart(Env):
    def __init__(self):
        # define environment
        self.quadrants = [False, False, False, False]
        # observation space
        self.lap_complete=False
        self.observation_space = Box(low=np.array([0,0,0,0,0]), high=np.array([40,40,40,40,40]), shape=(5,), dtype=np.float32)
        #action space
        self.action_space = Box(low=np.array([-1,-1],dtype=np.float32),high=np.array([1,1],dtype=np.float32),shape=(2,), dtype=np.float32)
        #duration
        self.start_time = rospy.get_time()
        self.track_time = 0
        # define reward
        self.reward = 0
        # define done
        self.done = False
        # define info
        self.info = {}
        # define action
        self.action = np.zeros(2, dtype=np.float32)
        # define observation
        self.observation =  np.zeros(5, dtype=np.float32)
        # define state
        self.state = (0,0,0,0,0) #(x,y,Vx,Vy,track_time)
        self.prev_x = 0
        self.prev_y = 0
        self.new_action = Twist()
        self.laserData = LaserScan()
        self.publisher_actions = rospy.Publisher(publish_topic_actuators, Twist)
        rospy.Subscriber(subscribe_topic_sensors, LaserScan, self.getObservation)
        rospy.Subscriber(subscribe_topic_position, Odometry, self.getState)

    def computequadrants(self,x,y):
        if x > 0 and y < 0:
            self.quadrants[0] = True
            
        elif x < 0 and y < 0 and self.quadrants[0]:
            self.quadrants[1] = True
            
        elif x < 0 and y > 0 and self.quadrants[1]:
            self.quadrants[2] = True
            
        elif x > 0 and y > 0 and self.quadrants[2]:
            self.quadrants[3] = True 
            

        
    def justPassedFinishLine(self,state):
        self.computequadrants(state[0],state[1])
        if state[0] > 0 and state[1] < 0 and self.quadrants[1]:
            return True
        else:
            return False

    def isFinished(self,state):#no need to pass state as it is a class variable
        if self.justPassedFinishLine(state) and all(self.quadrants):
            print('finish line passed')
            return True
        else:
            return False

    def getObservation(self,msg):
        #rospy.loginfo("log message on receipt of a new Lidar message.")
        self.observation = np.array([interval_mapping(msg.ranges[0],0,40,-1,1),interval_mapping(msg.ranges[1],0,40,-1,1),interval_mapping(msg.ranges[2],0,40,-1,1),interval_mapping(msg.ranges[3],0,40,-1,1),interval_mapping(msg.ranges[4],0,40,-1,1)],dtype=np.float32)
    def getState(self,msg):
        #rospy.loginfo("log message on receipt of a new Odometry message.")
        self.prev_x = self.state[0]
        self.prev_y = self.state[1]
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        Vx = msg.twist.twist.linear.x
        Vy = msg.twist.twist.linear.y
        self.state = (x,y,Vx,Vy,self.track_time)

        #denormaize action

    def step(self, action):
        # ros publish action
        self.new_action.linear.x = interval_mapping(action[0], -1, 1, -5, 30)
        self.new_action.angular.z = interval_mapping(action[1], -1, 1, -0.7, 0.7)
        #rospy.loginfo("log message on receipt of a new action. %f %f",self.new_action.linear.x,self.new_action.angular.z)
        self.publisher_actions.publish(self.new_action)
        self.track_time = rospy.get_time() - self.start_time
        # crashed ?
        #if isCrashed(self.state):
        if isCrashed(self.state):
            #rospy.loginfo("Crashed")
            self.done = True
            print('reward :',self.reward)
            self.reward += -100
            return self.observation, self.reward, self.done, self.info
        else:
            #print('not crashed')
            #self.reward += -1
            #for ever second deduct 1 reward
            #self.reward += -1/1000
            #calculate distance covered
            #self.reward+= np.sqrt((self.state[0]-self.prev_x)**2 + (self.state[1]-self.prev_y)**2)
            self.reward += np.sqrt(np.square(self.state[2])+np.square(self.state[3]))/100
        # is goal reached?
        if self.isFinished(self.state):
            print("lap over")
            #self.done = True
            self.reward += 1000
            self.quadrants = [False,False,False,False]
        return self.observation, self.reward, self.done, self.info
        
    def render(self):
        # already performed by flatland
        pass
    
    def reset(self):

        #delete model
        rospy.wait_for_service('delete_model')
        try:
            #rospy.loginfo("Reset : Deleting model")
            delete_model = rospy.ServiceProxy('delete_model', DeleteModel)
            rsp = delete_model('racing_cart')
            if not rsp.success:
                rospy.loginfo("Reset : Failed to delete model")
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

        #spawn model
            rospy.wait_for_service('spawn_model')
        try:
            #rospy.loginfo("Reset : Spawning model")
            spawn_model = rospy.ServiceProxy('spawn_model', SpawnModel)
            pose = Pose2D()
            pose.x = 2
            pose.y = 6
            pose.theta = -1.57
            rsp2 = spawn_model('racing_cart.yaml','racing_cart','',pose)
            if not rsp2.success:
                rospy.loginfo("Reset : Failed to spawn model")
            self.start_time = rospy.get_time()
            self.track_time = 0
            # define reward
            self.reward = 0
            # define done
            self.done = False
            # define info
            self.info = {}
            # define action
            self.action = np.zeros(2, dtype=np.float32)
            # define observation
            self.observation =  np.zeros(5, dtype=np.float32)
            # define state
            self.state = (0,0,0,0,0) #(x,y,Vx,Vy,track_time)
            self.prev_x = 0
            self.prev_y = 0
            self.quadrants = [False,False,False,False]
            self.new_action = Twist()
            self.laserData = LaserScan()
            return self.observation
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
        

#Initialize the ROS node
rospy.init_node('RL_vehicle_controller')
rospy.loginfo("Vehicle controller started.")
#Write publish and subscribe topics from parameter server into local variables
subscribe_topic_sensors = rospy.get_param("vehicle/sensor_topic")
publish_topic_actuators = rospy.get_param("vehicle/actuator_topic")
subscribe_topic_position = rospy.get_param("vehicle/position_topic")

rospy.loginfo("Vehicle controller subscribes to: %s", subscribe_topic_sensors)
rospy.loginfo("Vehicle controller publishes to: %s", publish_topic_actuators)


env = racing_cart()
check_env(env)

#model = PPO('MlpPolicy', env, verbose=1,seed=1337,tensorboard_log="./PPO_tensorboard/")
model= PPO.load("PPO_racing_cart_tb_2",env=env)
model.learn(total_timesteps=2e6,progress_bar=True,reset_num_timesteps=False,tb_log_name="third_run")
model.save("PPO_racing_cart_tb_3")
""" model= PPO.load("PPO_racing_cart4",env=env)
model.learn(total_timesteps=3e5,progress_bar=True,reset_num_timesteps=False)
model.save("PPO_racing_cart5")
model= PPO.load("PPO_racing_cart5",env=env)
model.learn(total_timesteps=3e5,progress_bar=True,reset_num_timesteps=False)
model.save("PPO_racing_cart6")
model= PPO.load("PPO_racing_cart6",env=env)
model.learn(total_timesteps=3e5,progress_bar=True,reset_num_timesteps=False)
model.save("PPO_racing_cart7") """
""" model.learn(total_timesteps=3e5,progress_bar=True,tb_log_name="second_run",reset_num_timesteps=False)
model.save("PPO_racing_cart2")
model.learn(total_timesteps=3e5,progress_bar=True,tb_log_name="third_run",reset_num_timesteps=False)
model.save("PPO_racing_cart3") """
""" rate = rospy.Rate(50) # 50hz
n_episodes = 10
for episode in range(n_episodes):
    obs = env.reset()
    done = False
    i = 0
    while  not rospy.is_shutdown():
        i = i+1
        print('episode :', episode+1 ,'step :' ,i)
        action = env.action_space.sample()
        #action = env.action_space.sample()
        obs, reward, done, info = env.step(action)
        print(reward)
        if done == True:
            break
        rate.sleep()
rospy.spin()
 """