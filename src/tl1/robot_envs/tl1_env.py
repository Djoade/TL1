#!/usr/bin/env python


from openai_ros import robot_gazebo_env


from sensor_msgs.msg import JointState
from gazebo_msgs.msg import LinkStates
from std_msgs.msg import Float64

import rospy

class TL1Env(robot_gazebo_env.RobotGazeboEnv):
    """Superclass for all Robot environments.
    """

    def __init__(self):
        """Initializes a new Robot environment.
        """
        # Variables that we give through the constructor.
        self.publishers_array = []
        self._right_pub = rospy.Publisher('/tl1/leg_right_controller/command', Float64, queue_size=1)
        self._left_pub = rospy.Publisher('/tl1/leg_right_controller/command', Float64, queue_size=1)
        self.publishers_array.append(self._right_pub)
        self.publishers_array.append(self._left_pub)
        
        rospy.Subscriber("/tl1/joint_states", JointState, self.joints_callback)
        rospy.Subscriber("/gazebo/link_states", LinkStates, self.links_callback)
        # Internal Vars
        self.controllers_list = ['tl1_joint_state_controller',
                                'tl1_leg_right_controller',
                                'tl1_leg_left_controller'
                                ]

        self.robot_name_space = "tl1"

        reset_controls_bool = False
        
        # We launch the init function of the Parent Class robot_gazebo_env.RobotGazeboEnv
        
        super(TL1Env, self).__init__(controllers_list=self.controllers_list,
                                                robot_name_space=self.robot_name_space,
                                                reset_controls=reset_controls_bool)

    # Methods needed by the RobotGazeboEnv
    # ----------------------------
    
    

    def _check_all_systems_ready(self):
        """
        Checks that all the sensors, publishers and other simulation systems are
        operational.
        """
        self.joints_pose = None
        self.height_pose = None
        while self.joints_pose is None and self.height_pose is None and not rospy.is_shutdown(): #SLOZHNO
            try:
                self.joints_pose = rospy.wait_for_message("/tl1/joint_states", JointState, timeout=1.0)
                self.height_pose = rospy.wait_for_message("/gazebo/link_states", LinkStates, timeout=1.0)
            except:
                rospy.logerr("Current cartpole_v0/joint_states not ready yet, retrying for getting joint_states")
        rospy.logdebug("ALL SYSTEMS READY")
        return True
    
    # Methods that the TrainingEnvironment will need to define here as virtual
    # because they will be used in RobotGazeboEnv GrandParentClass and defined in the
    # TrainingEnvironment.
    # ----------------------------
#
    def _set_init_pose(self):
        """Sets the Robot in its init pose
        """
        raise NotImplementedError()

    def _init_env_variables(self):
        """Inits variables needed to be initialised each time we reset at the start
        of an episode.
        """
        raise NotImplementedError()

    def _compute_reward(self, observations, done):
        """Calculates the reward to give based on the observations given.
        """
        raise NotImplementedError()

    def _set_action(self, action):
        """Applies the given action to the simulation.
        """
        raise NotImplementedError()

    def _get_obs(self):
        raise NotImplementedError()

    def _is_done(self, observations):
        """Checks if episode done based on observations given.
        """
        raise NotImplementedError()
#       
    # Methods that the TrainingEnvironment will need.
    # ----------------------------

    def move_joints(self, joints_array):
        joint_value_ll = Float64()
        joint_value_ll.data = joints_array[0]

        joint_value_rl = Float64()
        joint_value_rl.data = joints_array[1]
        
        self._left_pub.publish(joint_value_ll)
        self._right_pub.publish(joint_value_rl)


    def joints_callback(self, data):
        self.joints = data


    def links_callback(self, data):
        for i in range(len(data.name)):
            if data.name[i] == 'tl1::base_link':
                self.height = data.pose[i].position.z

    def init_internal_vars(self, init_pos_value):
        self.pos = [init_pos_value, init_pos_value]
        self.joints = None
        self.height = None