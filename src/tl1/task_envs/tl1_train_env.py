from tl1.robot_envs import tl1_env
import rospy
from gym import utils
from gym.envs.registration import register
from gym import error, spaces
import rospy
import math
import numpy

# The path is __init__.py of openai_ros, where we import the MovingCubeOneDiskWalkEnv directly
register(
        id='TL1TrainEnv-v0',
        entry_point='tl1.task_envs.tl1_train_env:TL1StandUp',
        #entry_point='tl1:task_envs.tl1_train_env.TL1StandUp',
    )

class TL1StandUp(tl1_env.TL1Env):
    def __init__(self):
        self.get_params()
        # Only variable needed to be set here
        
        self.action_space = spaces.Discrete(self.n_actions)
        
        self.sum_reward = 0
        # This is the most common case of Box observation type
        high = numpy.array([
            numpy.finfo(numpy.float32).max,
            numpy.finfo(numpy.float32).max,
            numpy.finfo(numpy.float32).max,
            numpy.finfo(numpy.float32).max,
            numpy.finfo(numpy.float32).max,
            numpy.finfo(numpy.float32).max,
            numpy.finfo(numpy.float32).max,
            numpy.finfo(numpy.float32).max,
            numpy.finfo(numpy.float32).max
            ])
            
        self.observation_space = spaces.Box(-high, high)
        
        # Variables that we retrieve through the param server, loded when launch training launch.
        
        # Here we will add any init functions prior to starting the MyRobotEnv
        super(TL1StandUp, self).__init__()

    def get_params(self):
        #get configuration parameters
        self.n_actions = rospy.get_param('/tl1/n_actions')
        self.pos_step = rospy.get_param('/tl1/pos_step')
        self.running_step = rospy.get_param('/tl1/running_step')
        self.init_pos = rospy.get_param('/tl1/init_pos')

    def _set_init_pose(self):
        """Sets the Robot in its init pose
        """
        self.init_internal_vars(self.init_pos)
        self.move_joints(self.pos)

    def _init_env_variables(self):
        """
        Inits variables needed to be initialised each time we reset at the start
        of an episode.
        :return:
        """
        self.sum_reward = 0


    def _set_action(self, action):
        """
        Move the robot based on the action variable given
        """
        # Take action
        if action == 1: #LL+
            rospy.loginfo("LL+")
            self.pos[0] += self.pos_step
        elif action == 3: #LL-
            rospy.loginfo("LL-")
            self.pos[0] -= self.pos_step
        elif action == 0: #RL+
            rospy.loginfo("RL+")
            self.pos[1] += self.pos_step
        elif action == 4: #RL-
            rospy.loginfo("RL-")
            self.pos[1] -= self.pos_step
        # elif action == 4: #RL+LL+
        #     rospy.loginfo("LL+RL+")
        #     self.pos[0] += self.pos_step
        #     self.pos[1] += self.pos_step
        # elif action == 5: #RL-LL-
        #     rospy.loginfo("LL-RL-")
        #     self.pos[0] -= self.pos_step
        #     self.pos[1] -= self.pos_step
        # elif action == 6: #RL+LL-
        #     rospy.loginfo("LL+LR-")
        #     self.pos[0] += self.pos_step
        #     self.pos[1] -= self.pos_step
        # elif action == 7: #RL-LL+
        #     rospy.loginfo("LL-RL+")
        #     self.pos[0] -= self.pos_step
        #     self.pos[1] += self.pos_step

          
       # Apply action to simulation.
        rospy.loginfo("MOVING TO POS=="+str(self.pos))

        self.move_joints(self.pos)
        rospy.logdebug("Wait for some time to execute movement, time="+str(self.running_step))
        rospy.sleep(self.running_step) #wait for some time
        rospy.logdebug("DONE Wait for some time to execute movement, time=" + str(self.running_step))

        # 3rd: pause simulation
        #rospy.logdebug("Pause SIM...")
        #self.gazebo.pauseSim()

    def _get_obs(self):

        data = self.joints
        obs = [data.position[0], data.position[1], self.x_orientation,self.y_orientation,self.z_orientation,self.w_orientation, self.x_position,self.y_position,self.z_position]
        print(obs[8])
        return numpy.array(obs)

    def _is_done(self, observations):
        """
        Decide if episode is done based on the observations
        """
        done = False
        data = self.joints
        
        if observations[8] < 0.4 and self.sum_reward < -25:
            done = True
        
        rospy.loginfo("FINISHED get _is_done")
        return done


    def _compute_reward(self, observations, done):
        """
        Return the reward based on the observations given
        """
        rospy.logdebug("START _compute_reward")
        
        if observations[8] > 0.4:
            reward = observations[8] * 10
            rew = 1
        else:
            reward = 0
            rew = -1
        self.sum_reward += rew
        print(observations[0])
        print(self.sum_reward)
        return reward
