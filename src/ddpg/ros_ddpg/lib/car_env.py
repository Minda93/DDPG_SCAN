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
car_r=20
# O_LC=40
O_LC=20
# O_LC=60
###################################################################
class CarEnv(object):
    viewer = None
    action_bound = [-math.pi, math.pi]
    point_bound =[30,370]
    v = 10
    
    def __init__(self):
        self.car_info=np.zeros(2, dtype=[('a', np.float32), ('i', np.float32)])
        self.car_info['a']=(20,300) 
        self.car_info['i']=(10,20)  #R=20 V=10
        self.o1_point=np.array([200,280])
        self.o2_point=np.array([150,150])
        self.goal_point=np.array([370,200])
        self.ran=math.pi
        self.obs_l=np.zeros(O_LC,dtype=np.float32)
        self.O_LC=O_LC
        self.state_old=np.ones(O_LC+2)

        self.car_store = [self.car_info['a'][0],self.car_info['a'][1]]
    def step(self, action):
        done = False
        r = 0.
        action = np.clip(action, *self.action_bound)
        self.car_info['a'][0]+=self.v*math.cos(action)
        self.car_info['a'][1]+=self.v*math.sin(action)
        
        self.car_info['a']=np.clip(self.car_info['a'],*self.point_bound)

        self.car_store.append(self.car_info['a'][0])
        self.car_store.append(self.car_info['a'][1])
        # print(self.car_store)
        # state

        v_goal=(self.goal_point-self.car_info['a'])
        self.obs_l[:]=self.obs_line()
        s=np.hstack((v_goal/400,self.obs_l/400))
#        s = self.car_info['a']/400
        # done and reward
        goal_car_dis=np.hypot(*(self.car_info['a']-self.goal_point)) 
        ob1_car_dis=np.hypot(*(self.car_info['a']-self.o1_point)) 
        ob2_car_dis=np.hypot(*(self.car_info['a']-self.o2_point)) 
        r=-goal_car_dis/4000

        if goal_car_dis<8+car_r:
            r += 1.
#         if goal_car_dis<25+car_r:
# #            done = True
#             r += 1.
        if ob1_car_dis<25+car_r:
#            done = True
            r += -1.
        if ob2_car_dis<25+car_r:
#            done = True
            r += -1.
        if self.car_info['a'][0]==30 or self.car_info['a'][0]==370 \
             or self.car_info['a'][1]==30 or self.car_info['a'][1]==370:
#                done = True
                r += -1.
                
        return s, r, done
    def obs_line(self,car_=None):
        obs_line=[]
        for i in np.linspace(-math.pi, math.pi,O_LC,endpoint=False):
            if car_ is None:
                car_point=self.car_info['a'].copy()
                car_=self.car_info['a'].copy()
            else:
                car_point=car_.copy()
                
            for j in np.linspace(1,400,20):
                car_point[0]=car_[0]+j*math.cos(i)
                car_point[1]=car_[1]+j*math.sin(i)
                car_point=np.clip(car_point,1,399)
                ob1_car_dis=np.hypot(*(car_point-self.o1_point)) 
                ob2_car_dis=np.hypot(*(car_point-self.o2_point)) 
                if ob1_car_dis<25 or ob2_car_dis<25 \
                    or car_point[1]==1 or car_point[1]==399:
#                    or car_point[0]==1 or car_point[0]==399 \
                        break
            obs_line.append(j)
        
        return obs_line
    def step_state(self,action):
        done = False
        r = 0.
        action = np.clip(action, *self.action_bound)
        self.car_info['a'][0]+=self.v*math.cos(action)
        self.car_info['a'][1]+=self.v*math.sin(action)

        self.car_info['a']=np.clip(self.car_info['a'],*self.point_bound)
        
        # state

        v_goal=(self.goal_point-self.car_info['a'])
        self.obs_l[:]=self.obs_line()
        s=np.hstack((v_goal/400,self.obs_l/400))
        # done and reward
        goal_car_dis=np.hypot(*(self.car_info['a']-self.goal_point)) 
        r=-goal_car_dis/4000
        
        #use obs_l to find reward
        line_count=1
        j=0
        crash=False
        for i in self.obs_l:
            if i<25:
                j+=1
                if j>line_count:
                    crash=True;
                    break
            else:
                j=0
            
        if goal_car_dis<8+car_r:
            r += 1.
        # if goal_car_dis<25+car_r:
        #     r += 1.
        if crash is True:
            r += -1.
        if self.car_info['a'][0]==30 or self.car_info['a'][0]==370 \
             or self.car_info['a'][1]==30 or self.car_info['a'][1]==370:
                r += -1.
        return s, r, done
    def state_reward(self,state,action,rate):
        r = 0.
        action = np.clip(action, *self.action_bound)
        self.car_info['a'][0]=200
        self.car_info['a'][1]=200
        s=self.state_old
        # state

        self.obs_l[:]=state[2:]*rate/2
        self.goal_point[:]=[200,200]+state[:2]*rate/2
        self.o1_point[:]=[200,200]+state[:2]*rate/2
        self.o2_point[:]=[500,500]
        # done and reward
        goal_car_dis=np.hypot(*(state[:2]*rate)) 
        r=-goal_car_dis/4000
        
        #use obs_l to find reward
        line_count=1
        j=0
        crash=False
        for i in self.obs_l:
            if i<25:
                j+=1
                if j>line_count:
                    crash=True;
                    break
            else:
                j=0
            
        if goal_car_dis<8+car_r:
            r += 1.
        # if goal_car_dis<25+car_r:
        #     r += 1.
        if crash == True:
            r += -1.
        self.state_old=state
        return  s,r,state,crash

    def reset(self):
    
#        self.car_info['a']=(30,300)
        self.o1_point[:]=np.random.rand(2)*(100,200)+(100,30)
        self.o2_point[:]=np.random.rand(2)*(100,150)+(100,200)
        self.car_info['a']=np.random.rand(2)*(1,340)+30
        self.goal_point[:]=np.random.rand(2)*(300,370)+(100,30)

        # case 1
        # self.o1_point[:] = np.array([100,280])
        # self.o2_point[:] = np.array([260,150])
        # self.car_info['a'] = np.array([20,300]) 
        # self.goal_point[:] = np.array([370,200])

        # case 2
        # self.o1_point[:] = np.array([200,250])
        # self.o2_point[:] = np.array([200,180])
        # self.car_info['a'] = np.array([20,220]) 
        # self.goal_point[:] = np.array([370,200])

        self.obs_l[:]=self.obs_line()
        s=np.hstack((self.car_info['a']/400,self.obs_l/400))
        v_goal=(self.goal_point-self.car_info['a'])
        self.obs_l[:]=self.obs_line()
        s=np.hstack((v_goal/400,self.obs_l/400))

        self.car_store[:] = [self.car_info['a'][0], self.car_info['a'][1]]
        
        return s
    
    def render(self):
        if self.viewer is None:
            print('fuck')
            self.viewer = Viewer(self.goal_point,self.car_info['a'],self.o1_point,self.o2_point,self.obs_l,self.car_store)
        # self.viewer.render(self.car_store)
        self.viewer.render()
    def sample_action(self):
        self.ran-=0.1
        if self.ran<-math.pi:
            self.ran=math.pi
#        return np.random.rand(2)+2  # two radians
        return self.ran
        

class Viewer(pyglet.window.Window):
    bar_thc = 5

    def __init__(self,goal_point,car_point,o1_point,o2_point,obs_line,car_store):
        # vsync=False to not use the monitor FPS, we can speed up training
        super(Viewer, self).__init__(width=400, height=400, resizable=True, caption='gooood_car', vsync=False)
        self.car_point_=car_point
        self.obs_line=obs_line
        self.o1_point=o1_point
        self.o2_point=o2_point
        self.goal_point=goal_point
        self.car_store = car_store
        
        o1_v=np.hstack([self.o1_point-25,self.o1_point[0]-25,self.o1_point[1]+25,self.o1_point+25,self.o1_point[0]+25,self.o1_point[1]-25])
        o2_v=np.hstack([self.o2_point-25,self.o2_point[0]-25,self.o2_point[1]+25,self.o2_point+25,self.o2_point[0]+25,self.o2_point[1]-25])
        goal_v=np.hstack([self.goal_point-8,self.goal_point[0]-8,self.goal_point[1]+8,self.goal_point+8,self.goal_point[0]+8,self.goal_point[1]-8])
        # goal_v=np.hstack([self.goal_point-25,self.goal_point[0]-25,self.goal_point[1]+25,self.goal_point+25,self.goal_point[0]+25,self.goal_point[1]-25])
        pyglet.gl.glClearColor(1, 1, 1, 1)
        #        GL_POINTS  GL_LINES GL_LINE_STRIP GL_LINE_LOOP GL_POINTS
        print(o1_v)
        self.batch = pyglet.graphics.Batch()    # display whole batch at once
        self.goal = self.batch.add(
            4, pyglet.gl.GL_QUADS, None,    # 4 corners
            ('v2f', goal_v),
            ('c3B', (86, 109, 249) * 4))    # color
        self.obs_1 = self.batch.add(
            4, pyglet.gl.GL_QUADS, None,
            ('v2f', o1_v),
            ('c3B', (249, 86, 86) * 4,))
        self.obs_2 = self.batch.add(
            4, pyglet.gl.GL_QUADS, None,
            ('v2f', o2_v ),
            ('c3B', (249, 86, 86) * 4,))    # color
        car_dot=self.makeCircle(200,car_r,*self.car_point_)
        self.car = self.batch.add(
            int(len(car_dot)/2), pyglet.gl.GL_LINE_LOOP, None,
            ('v2f', car_dot), ('c3B', (0, 0, 0) * int(len(car_dot)/2)))

        line_dot=self.linedot()
        self.o_line=self.batch.add(
            int(len(line_dot)/2), pyglet.gl.GL_LINES, None,
            ('v2f', line_dot), ('c3B', (0, 0, 0) * int(len(line_dot)/2)))

        # self.car_line=self.batch.add(
        #     int(len(self.car_store)/2), pyglet.gl.GL_POINTS, None,
        #     ('v2f', self.car_store), ('c3B', (249, 86, 86)*int(len(self.car_store)/2)))

        self.car_line=self.batch.add(
            int(len(self.car_store)/2), pyglet.gl.GL_LINES, None,
            ('v2f', self.car_store), ('c3B', (249, 86, 86)*int(len(self.car_store)/2)))

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
            l_dot=self.car_point_.copy()
            line_dot_v.append(l_dot.copy())
            l_dot[0]+=self.obs_line[j]*math.cos(i)
            l_dot[1]+=self.obs_line[j]*math.sin(i)
            line_dot_v.append(l_dot)
        return np.hstack(line_dot_v)
    
    def render(self):
        # self.car_store = car_store
        self._update_car()
        self.switch_to()
        self.dispatch_events()
        self.dispatch_event('on_draw')
        self.flip()
        
    def on_draw(self):
        self.clear()
        self.batch.draw()
        

    def _update_car(self):
        
        self.car_line.delete()

        # self.car_line=self.batch.add(
        #     int(len(self.car_store)/2), pyglet.gl.GL_POINTS, None,
        #     ('v2f', self.car_store), ('c3B', (249, 86, 86)*int(len(self.car_store)/2)))
        
        self.car_line=self.batch.add(
            int(len(self.car_store)/2), pyglet.gl.GL_LINES, None,
            ('v2f', self.car_store), ('c3B', (249, 86, 86)*int(len(self.car_store)/2)))

        self.car_line.vertices = self.car_store
        print(len(self.car_store))
        car_dot=self.makeCircle(200,car_r,*self.car_point_)
        self.car.vertices = car_dot 
        line_dot=self.linedot()
        self.o_line.vertices=line_dot
        o1_v=np.hstack([self.o1_point-25,self.o1_point[0]-25,self.o1_point[1]+25,self.o1_point+25,self.o1_point[0]+25,self.o1_point[1]-25])
        o2_v=np.hstack([self.o2_point-25,self.o2_point[0]-25,self.o2_point[1]+25,self.o2_point+25,self.o2_point[0]+25,self.o2_point[1]-25])
        # goal_v=np.hstack([self.goal_point-25,self.goal_point[0]-25,self.goal_point[1]+25,self.goal_point+25,self.goal_point[0]+25,self.goal_point[1]-25])
        goal_v=np.hstack([self.goal_point-8,self.goal_point[0]-8,self.goal_point[1]+8,self.goal_point+8,self.goal_point[0]+8,self.goal_point[1]-8])
        self.obs_1.vertices=o1_v
        self.obs_2.vertices=o2_v
        self.goal.vertices=goal_v
    def on_close(self):
        self.close()
    def on_mouse_motion(self, x, y, dx, dy):
        self.goal_point[0] = x
        self.goal_point[1] = y
        
if __name__ == '__main__':
    env = CarEnv()
    s=env.reset()
    a=0
    while True:
        s=env.reset()
        for i in range(100):
            env.render()
#            env.step_state(env.sample_action())
            _,r,s_,crash=env.state_reward(s,a,400)
#            time.sleep(1)

#    pyglet.on_close()