#!/usr/bin/env python
import rospy

import gym
import keras
import numpy as np
import random

from gym import wrappers
from keras.models import Sequential
from keras.layers import Dense
from keras.optimizers import Adam

from collections import deque

from openai_ros.msg import RLExperimentInfo
# import our training environment
import sys
sys.path.append('/home/dkozlov/catkin_ws/src/tl1/src')

from tl1.task_envs import tl1_train_env



ACTIONS_DIM = 4
OBSERVATIONS_DIM = 9
MAX_ITERATIONS = 1000
LEARNING_RATE = 0.001

NUM_EPOCHS = 50

GAMMA = 0.99
REPLAY_MEMORY_SIZE = 1000
NUM_EPISODES = 1000
TARGET_UPDATE_FREQ = 100
MINIBATCH_SIZE = 32

RANDOM_ACTION_DECAY = 0.99
INITIAL_RANDOM_ACTION = 1

class ReplayBuffer():

  def __init__(self, max_size):
    self.max_size = max_size
    self.transitions = deque()

  def add(self, observation, action, reward, observation2):
    if len(self.transitions) > self.max_size:
      self.transitions.popleft()
    self.transitions.append((observation, action, reward, observation2))

  def sample(self, count):
    return random.sample(self.transitions, count)

  def size(self):
    return len(self.transitions)

def get_q(model, observation):
  np_obs = np.reshape(observation, [-1, OBSERVATIONS_DIM])
  return model.predict(np_obs)

def train(model, observations, targets):
  # for i, observation in enumerate(observations):
  #   np_obs = np.reshape(observation, [-1, OBSERVATIONS_DIM])
  #   print "t: {}, p: {}".format(model.predict(np_obs),targets[i])
  # exit(0)

  np_obs = np.reshape(observations, [-1, OBSERVATIONS_DIM])
  np_targets = np.reshape(targets, [-1, ACTIONS_DIM])

  #model.fit(np_obs, np_targets, epochs=1, verbose=0)
  model.fit(np_obs, np_targets, nb_epoch=1, verbose=0)

def predict(model, observation):
  np_obs = np.reshape(observation, [-1, OBSERVATIONS_DIM])
  return model.predict(np_obs)

def get_model():
  model = Sequential()
  model.add(Dense(16, input_shape=(OBSERVATIONS_DIM, ), activation='relu'))
  model.add(Dense(16, input_shape=(OBSERVATIONS_DIM,), activation='relu'))
  model.add(Dense(4, activation='linear'))

  model.compile(
    optimizer=Adam(lr=LEARNING_RATE),
    loss='mse',
    metrics=[],
  )

  return model

def update_action(action_model, target_model, sample_transitions):
  random.shuffle(sample_transitions)
  batch_observations = []
  batch_targets = []

  for sample_transition in sample_transitions:
    old_observation, action, reward, observation = sample_transition

    targets = np.reshape(get_q(action_model, old_observation), ACTIONS_DIM)
    targets[action] = reward
    if observation is not None:
      predictions = predict(target_model, observation)
      new_action = np.argmax(predictions)
      targets[action] += GAMMA * predictions[0, new_action]

    batch_observations.append(old_observation)
    batch_targets.append(targets)

  train(action_model, batch_observations, batch_targets)

def main():
  steps_until_reset = TARGET_UPDATE_FREQ
  random_action_probability = INITIAL_RANDOM_ACTION

  # Initialize replay memory D to capacity N
  replay = ReplayBuffer(REPLAY_MEMORY_SIZE)

  # Initialize action-value model with random weights
  action_model = get_model()

  # Initialize target model with same weights
  #target_model = get_model()
  #target_model.set_weights(action_model.get_weights())

  env = gym.make('TL1TrainEnv-v0')
  env = wrappers.Monitor(env, '/tmp/tl1-experiment-1', force=True)

  for episode in range(NUM_EPISODES):
    observation = env.reset()

    for iteration in range(MAX_ITERATIONS):
      random_action_probability *= RANDOM_ACTION_DECAY
      random_action_probability = max(random_action_probability, 0.1)
      old_observation = observation

      # We dont support render in openai_ros
      """
      if episode % 1 == 0:
        env.render()
      """
      
      if np.random.random() < random_action_probability:
        action = np.random.choice(range(ACTIONS_DIM))
      else:
        q_values = get_q(action_model, observation)
        action = np.argmax(q_values)

      observation, reward, done, info = env.step(action)
      

      if done:
        print 'Episode {}, iterations: {}'.format(
          episode,
          iteration
        )

        # print action_model.get_weights()
        # print target_model.get_weights()

        #print 'Game finished after {} iterations'.format(iteration)
        reward = -200
        replay.add(old_observation, action, reward, None)
        
        break

      replay.add(old_observation, action, reward, observation)

      if replay.size() >= MINIBATCH_SIZE:
        sample_transitions = replay.sample(MINIBATCH_SIZE)
        update_action(action_model, action_model, sample_transitions)
        steps_until_reset -= 1



if __name__ == "__main__":
    rospy.init_node('tl1_mbalunovic_algorithm', anonymous=True, log_level=rospy.ERROR)
    main()