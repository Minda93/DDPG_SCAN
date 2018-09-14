# -*- coding: utf-8 -*-
"""
Created on Fri Feb 16 15:40:27 2018

@author: user
"""


import numpy as np
import time
import sys
import math
import pyglet


UNIT = 10   # pixels
MAZE_H = 4  # grid height
MAZE_W = 4  # grid width

# ==========================
# car_info

CAR_INIT_POSITION = (20,300) # car init position
CAR_RADIUS = 20  # car radius
CAR_VEC = 10     # car velocity

O_LC=20          # number of scan line    
# O_LC=40
# O_LC=60

OBSTACLE_SIZE = 25
GOAL_SIZE = 8

ERROR_DIS = 25
GOAL_ERROR = 8
# ==========================
# obstacle info

# OBSTACLE = []   # 1
OBSTACLE = [[200,280],[150,150]]   # 2
# OBSTACLE = []   # 3

# ==========================
# goal info
GOAL = [370,200]


# ==========================
# FLAG
CAR_RANDOM_FLAG = False
OBSTACLE_RANDOM_FLAG = False
GOAL_RANDOM_FLAG = False

###################################################################
class CarEnv(object):
    def __init__(self):
        self.viewer = None
        
        self.action_bound = [-math.pi, math.pi]
        
        self.point_bound = [30,370]
        self.car_info = np.zeros(2, dtype=[('a', np.float32), ('i', np.float32)])
        self.car_info['a'] = CAR_INIT_POSITION
        self.car_info['i'] = (CAR_VEC,CAR_RADIUS)
        self.car_trajectory = [self.car_info['a'][0],self.car_info['a'][1]]

        self.obstacle_num = 2
        self.obstacle = None
        self.goal = np.array(GOAL)

        self.obs_l = np.zeros(O_LC,dtype=np.float32)
        self.O_LC = O_LC
        self.state_old = np.ones(O_LC+2)

        self.Init_Car(CAR_RANDOM_FLAG)
        self.Init_Obstacle(OBSTACLE_RANDOM_FLAG)
        self.Init_Goal(GOAL_RANDOM_FLAG)

    def Init_Car(self,reset = False):
        if(reset):
            self.car_info['a'][:] = np.random.rand(2)*(1,340)+30
        else:
            self.car_info['a'][:] = CAR_INIT_POSITION

    def Init_Obstacle(self,reset = False):
        if(reset):
            obstacle = []
            for i in range(self.obstacle_num):
                pos_ = np.random.rand(2)*(400,400)
                if(i == 0):
                    obstacle.append(pos_)
                else:
                    flag = 1
                    while(flag == 1):
                        for pos in obstacle:
                            if(math.hypot(pos[0] - pos_[0], pos[1] - pos_[1]) < 60):
                                flag = 1
                                break
                            else:
                                flag = 0
                        if(flag == 1):
                            pos_ = np.random.rand(2)*(400,400)
                    obstacle.append(pos_)
            self.obstacle = np.array(obstacle)
        else:
            self.obstacle = np.array(OBSTACLE)
    
    def Init_Goal(self,reset = False):
        if(reset):
            self.goal[:] = np.random.rand(2)*(300,370)+(100,30)
        else:
            self.goal[:] = np.array(GOAL)
    
    def step(self,action):
        done = False
        reward = 0.0
        action = np.clip(action, *self.action_bound)

        self.car_info['a'][0] += self.car_info['i'][0]*math.cos(action)
        self.car_info['a'][1] += self.car_info['i'][0]*math.sin(action)
        self.car_info['a'] = np.clip(self.car_info['a'],*self.point_bound)

        self.car_trajectory.append(self.car_info['a'][0])
        self.car_trajectory.append(self.car_info['a'][1])

        # state
        v_goal = (self.goal-self.car_info['a'])
        self.obs_l[:] = self.obs_line()
        state = np.hstack((v_goal/400,self.obs_l/400))
        
        # done and reward
        goal_car_dis = np.hypot(*(self.car_info['a']-self.goal))
        
        obstacle_car_dis = []
        for obstacle in self.obstacle: 
            obstacle_car_dis.append(np.hypot(*(self.car_info['a']-obstacle)))
        obstacle_car_dis = np.array(obstacle_car_dis)

        reward = -goal_car_dis/4000

        if(goal_car_dis < GOAL_ERROR+self.car_info['i'][1]):
            reward += 1.
            # done = True
        
        for distance in obstacle_car_dis:
            if(distance < ERROR_DIS+self.car_info['i'][1]):
                reward += -1.
                # done = True
        
        if(self.car_info['a'][0] == 30 or self.car_info['a'][0] == 370 \
             or self.car_info['a'][1] == 30 or self.car_info['a'][1] == 370):
                # done = True
                reward += -1.

        return state,reward,done
    
    def obs_line(self,car_ = None):
        obs_line = []
        for i in np.linspace(self.action_bound[0],self.action_bound[1],O_LC,endpoint=False):
            if car_ is None:
                car = self.car_info['a'].copy()
                car_ = self.car_info['a'].copy()
            else:
                car = car_.copy()
            
            flag = 0
            for j in np.linspace(1,400,20):
                car[0] = car_[0]+j*math.cos(i)
                car[1] = car_[1]+j*math.sin(i)
                car = np.clip(car,1,399)

                for obstacle in self.obstacle:
                    obstacle_car_dis = np.hypot(*(car-obstacle)) 
                    if(car[1] == 1 or car[1] == 399 or obstacle_car_dis < ERROR_DIS):
                        flag = 1
                        break

                if(flag == 1):
                    break
            obs_line.append(j)

        return obs_line
    
    def step_state(self,action):
        done = False
        reward = 0.
        action = np.clip(action,*self.action_bound)

        self.car_info['a'][0] += self.car_info['i'][0]*math.cos(action)
        self.car_info['a'][1] += self.car_info['i'][0]*math.sin(action)
        self.car_info['a'] = np.clip(self.car_info['a'],*self.point_bound)

        # state 
        v_goal = (self.goal-self.car_info['a'])
        self.obs_l[:] = self.obs_line()
        state = np.hstack((v_goal/400,self.obs_l/400))

        # done and reward
        goal_car_dis = np.hypot(*(self.car_info['a']-self.goal)) 
        reward = -goal_car_dis/4000

        #use obs_l to find reward
        line_count = 1
        j = 0
        crash = False
        for i in self.obs_l:
            if(i < ERROR_DIS):
                j += 1
                if j > line_count:
                    crash = True;
                    break
            else:
                j = 0
        
        if(goal_car_dis < GOAL_ERROR+self.car_info['i'][1]):
            reward += 1.
        if(crash):
            reward += -1.

        if self.car_info['a'][0] == 30 or self.car_info['a'][0] == 370 \
             or self.car_info['a'][1] == 30 or self.car_info['a'][1] == 370:
                reward += -1.
        
        return state, reward, done

    def reset(self,reset = False):
        self.Init_Car(reset)
        self.Init_Obstacle(reset)
        self.Init_Goal(reset)

        self.obs_l[:] = self.obs_line()
        state = np.hstack((self.car_info['a']/400,self.obs_l/400))
        v_goal = (self.goal-self.car_info['a'])
        self.obs_l[:] = self.obs_line()
        state = np.hstack((v_goal/400,self.obs_l/400))

        self.car_trajectory[:] = [self.car_info['a'][0], self.car_info['a'][1]]

        return state

    def render(self):
        if self.viewer is None:
            self.viewer = Viewer(self.car_info['a'],self.obstacle,self.goal,self.obs_l,self.car_trajectory)
        self.viewer.render(self.obstacle)
        
class Viewer(pyglet.window.Window):
    def __init__(self,car,obstacle,goal,obs_line,trajectory):
        # vsync=False to not use the monitor FPS, we can speed up training
        super(Viewer, self).__init__(width=400, height=400, resizable=True, caption='gooood_car', vsync=False)

        self.car = car
        self.obstacle = obstacle
        self.obstacleNum = len(obstacle)
        self.goal = goal
        self.obs_line = obs_line
        self.car_trajectory = trajectory

        pyglet.gl.glClearColor(1, 1, 1, 1)

        self.batch = pyglet.graphics.Batch() 
        self.simCar = None
        self.simLine = None
        self.simTrajectory = None
        self.simObstacle = []
        self.simGoal = None

        self.Init_SimCar_And_simLine_And_SimGoal()
        self.Init_SimObstacle()
        self.Init_SimTrajectory()

    def Init_SimCar_And_simLine_And_SimGoal(self):
        
        car_dot = self.makeCircle(200,CAR_RADIUS,*self.car)
        self.simCar = self.batch.add(
            int(len(car_dot)/2), pyglet.gl.GL_LINE_LOOP, None,
            ('v2f', car_dot), ('c3B', (0, 0, 0) * int(len(car_dot)/2)))

        line_dot = self.linedot()
        self.simLine = self.batch.add(
            int(len(line_dot)/2), pyglet.gl.GL_LINES, None,
            ('v2f', line_dot), ('c3B', (0, 0, 0) * int(len(line_dot)/2)))
        
        goal_v = np.hstack([self.goal-GOAL_SIZE,self.goal[0]-GOAL_SIZE,self.goal[1]+GOAL_SIZE,self.goal+GOAL_SIZE,self.goal[0]+GOAL_SIZE,self.goal[1]-GOAL_SIZE])
        self.simGoal = self.batch.add(
            4, pyglet.gl.GL_QUADS, None,    # 4 corners
            ('v2f', goal_v),
            ('c3B', (86, 109, 249) * 4))    # color
        

    def Init_SimObstacle(self):
        obstacle_v = []
        for item in self.obstacle:
            obstacle_v.append([item[0]-OBSTACLE_SIZE,item[1]-OBSTACLE_SIZE,item[0]-OBSTACLE_SIZE,item[1]+OBSTACLE_SIZE,item[0]+OBSTACLE_SIZE,item[1]+OBSTACLE_SIZE,item[0]+OBSTACLE_SIZE,item[1]-OBSTACLE_SIZE])
        # obstacle_v = np.array(obstacle_v)

        # if(self.simObstacle != []):
        #     for i in range(len(self.simObstacle)):
        #         self.simObstacle[i].delete()
        
        for obstacle in obstacle_v:
            # print(obstacle)
            self.simObstacle.append(self.batch.add(
            4, pyglet.gl.GL_QUADS, None,
            ('v2f', obstacle),
            ('c3B', (249, 86, 86) * 4,)))
        
    def Init_SimTrajectory(self):
        if(self.simTrajectory is None):
            self.simTrajectory = self.batch.add(
            int(len(self.car_trajectory)/2), pyglet.gl.GL_LINES, None,
            ('v2f', self.car_trajectory), ('c3B', (249, 86, 86)*int(len(self.car_trajectory)/2)))
        else:
            self.simTrajectory.delete()
            self.simTrajectory = self.batch.add(
            int(len(self.car_trajectory)/2), pyglet.gl.GL_LINES, None,
            ('v2f', self.car_trajectory), ('c3B', (249, 86, 86)*int(len(self.car_trajectory)/2)))
    
    def makeCircle(self,numPoints,r,c_x,c_y):
        verts = []
        for i in range(numPoints):
            angle = math.radians(float(i)/numPoints * 360.0)
            x = r*math.cos(angle) + c_x
            y = r*math.sin(angle) + c_y
            verts += [x,y]
        return verts        
    
    def linedot(self):
        line_dot_v=[]
        for i, j in zip(np.linspace(-math.pi, math.pi,O_LC,endpoint=False),range(O_LC)):
            l_dot = self.car.copy()
            line_dot_v.append(l_dot.copy())
            l_dot[0] += self.obs_line[j]*math.cos(i)
            l_dot[1] += self.obs_line[j]*math.sin(i)
            line_dot_v.append(l_dot)
        return np.hstack(line_dot_v)
    
    def render(self,obstacle):
        self.obstacle = obstacle
        self._update_car()
        self.switch_to()
        self.dispatch_events()
        self.dispatch_event('on_draw')
        self.flip()
        
    def on_draw(self):
        self.clear()
        self.batch.draw()
    
    def _update_car(self):
        
        car_dot = self.makeCircle(200,CAR_RADIUS,*self.car)
        self.simCar.vertices = car_dot

        line_dot = self.linedot()
        self.simLine.vertices = line_dot

        if(len(self.simObstacle) != self.obstacleNum):
            self.Init_SimObstacle()
        else:
            i = 0
            for item in self.obstacle:
                obstacle_v = [item[0]-OBSTACLE_SIZE,item[1]-OBSTACLE_SIZE,item[0]-OBSTACLE_SIZE,item[1]+OBSTACLE_SIZE,item[0]+OBSTACLE_SIZE,item[1]+OBSTACLE_SIZE,item[0]+OBSTACLE_SIZE,item[1]-OBSTACLE_SIZE]
                self.simObstacle[i].vertices = obstacle_v
                i += 1

        self.Init_SimTrajectory()

        goal_v = np.hstack([self.goal-GOAL_SIZE,self.goal[0]-GOAL_SIZE,self.goal[1]+GOAL_SIZE,self.goal+GOAL_SIZE,self.goal[0]+GOAL_SIZE,self.goal[1]-GOAL_SIZE])

        self.simGoal.vertices = goal_v

    def on_close(self):
        self.close()

    def on_mouse_motion(self, x, y, dx, dy):
        self.goal[0] = x
        self.goal[1] = y


            






      



###################################################################
def main():
    env = CarEnv()
    print(env.obstacle)
    print('========')
    env.step(0)

if __name__ == '__main__':
    main()