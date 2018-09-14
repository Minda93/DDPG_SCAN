#!/usr/bin/env python3
# -*- coding: utf-8 -*-+

import sys
import numpy as np
import math
import time

# rostopic lib 
from geometry_msgs.msg import Twist

# lib
# from lib.car_env import CarEnv
from lib.car_env_new import CarEnv
from lib.DDPG import DDPG
from lib.nodehandle import NodeHandle

# FLAG
FIRST_TRAIN = False

# Behavior
ROBOT = 0
TRAIN = 1
VIEW_RESULT = 2

class Strategy(object):
    def __init__(self):
        self._param = NodeHandle()
        self._env = CarEnv()
        self._ddpg = None

        self._state = None
        self._action = None
        
        # self._behavior = ROBOT
        # self._behavior = TRAIN
        self._behavior = VIEW_RESULT

        self.Init_DDPG()

        if(self._behavior == TRAIN):
            self.Init_Train()

        if(self._behavior == VIEW_RESULT):
            self.Init_View_Result()

    def Init_DDPG(self):
        s_dim = 2+self._env.O_LC
        a_dim = 1
        a_bound = self._env.action_bound[1]
        self._ddpg = DDPG(a_dim, s_dim, a_bound)
        if(FIRST_TRAIN == False):
            self._ddpg.restore()

    def Process(self):
        if(self._behavior == ROBOT):
            if(self._param.scan):
                self.Calculate_State()
                self._action = self._ddpg.choose_action(self._state)
                x,y = self.Calculate_Vel()
                self.Robot_Vel([-y,x])
                print(x,-y)
            else:
                print('No Scan!!!')
                self.Robot_Stop()
        elif(self._behavior == TRAIN):
            self.Simulator_Train()
        elif(self._behavior == VIEW_RESULT):
            self.Eval_Viewer()
            

    def Calculate_State(self):
        line = np.array(self._param.scan,dtype=np.float32)/1000
        num_change = self._param.finalAng*3-270
        go_where_x = math.cos(num_change*math.pi/180)
        go_where_y = math.sin(num_change*math.pi/180)

        state = []
        #state[0:1]=np.array([go_where_x,go_where_y])
        state[0:1]=np.array([1,0])
        state[2:11]=line[60:120:6]
        state[12:21]=line[0:60:6]
        # print(len(state))
        self._state=np.array(state)

    def Calculate_Vel(self): 
        x = self.maxVel*math.cos(self._action)
        y = self.maxVel*math.sin(self._action)
        return x,y

    def Robot_Stop(self):
        vel = Twist()
        vel.linear.x = 0
        vel.linear.y = 0
        vel.angular.z = 0
        self._param.pub_cmdvel.publish(vel)

    def Robot_Vel(self,vec):
        vel = Twist()
        vel.linear.x = vec[0]
        vel.linear.y = vec[1]
        self._param.pub_cmdvel.publish(vel)
    # ==============================================
    def Init_Train(self):
        self.t1_ = time.time()
        self._render = False
        self._var = 1
        self._memoryCapacity = 10000
        self._maxEpisodes = 1000
        self._maxEpSteps = 200

        self.runningR = []
        self.goodJob = 0
        self.times = 0
    def Simulator_Train(self):
        if(self.times < self._maxEpisodes):
            t1 = time.time()
            self._state = self._env.reset()
            ep_reward = 0
            for i in range(self._maxEpSteps):
                if(self._render):
                    self._env.render()
                
                # Add exploration noise
                self._action = self._ddpg.choose_action(self._state)
                self._action = np.clip(np.random.normal(self._action,self._var), *self._env.action_bound)
                state,reward,done = self._env.step_state(self._action)
                self._ddpg.store_transition(self._state, self._action, reward / 10, state)
                if self._ddpg.pointer > self._memoryCapacity:
                    self._var *= .9995    # decay the action randomness
                self._ddpg.learn()

                self._state = state.copy()
                ep_reward += reward
                if(i == self._maxEpSteps-1 or done == True):
                    print('Episode:', self.times, ' Reward: %i' % int(ep_reward), 'Explore: %.2f' % self._var,'Running time: ', time.time() - t1 )
                    if(len(self.runningR) == 0):
                        self.runningR.append(ep_reward)
                    else:
                        self.runningR.append(self.runningR[-1]*0.9+ep_reward*0.1)
                    if(ep_reward > 30 and self.times > 200):
                        self._render = False
                        self._var = 0
                        self.goodJob += 1
                        print(self.goodJob)
                    else:
                        if(self.goodJob > 0):
                            self.goodJob -= 1
                    break
            self.times += 1
            if (self.goodJob > 40):
                print('Running time: ', time.time() - self.t1_)
                self._ddpg.save()
            if(self.goodJob > 41):
                sys.exit()
        else:
            print('fuck')
            self._ddpg.save()
            sys.exit()
        
    # ==============================================
    def Init_View_Result(self):
        # self._ddpg.restore()
        self._env.render()
        self._env.viewer.set_vsync(True)
        self._state = self._env.reset()
    def Eval_Viewer(self):
        self._state = self._env.reset(True)/400
        ep_reward = 0
        for j in range(100):
            self._env.render()
            self._action = self._ddpg.choose_action(self._state)
            self._state,reward,done = self._env.step(self._action)
            # print(5*math.cos(self._action),5*math.sin(self._action))
            ep_reward += reward
            if(j == 99 or done == True):
                # print(' Reward: %i' % int(ep_reward) )
                # print(self._env.car_store)
                # sys.exit()
                break

        

