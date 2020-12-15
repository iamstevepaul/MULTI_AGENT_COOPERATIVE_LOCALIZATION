from Environment import Environment
from Robot import Robot
import numpy as np
import random
import matplotlib.pyplot as plt

max_rectangle = 100
min_rectangle = 0
corners = [[min_rectangle,min_rectangle], [min_rectangle,max_rectangle], [100, min_rectangle], [max_rectangle,max_rectangle]]
corners = np.array(corners)

n_landmarks = 10
n_agents = 5
n_waypoints = 2

v_max = 10

start_points = np.array([[random.random(), random.random()] for i in range(n_agents)])*max_rectangle
velocity = (np.array([[random.random(), random.random()] for i in range(n_agents)])-0.5)*2*v_max

landmarks = np.array([[random.random(), random.random()] for i in range(n_landmarks)])*max_rectangle
total_time = 10
n_time_Steps = 1000
dt = total_time/n_time_Steps
time = 0
robots = [Robot(x=start_points[i,0], y=start_points[i,1], vx=velocity[i,0], vy=velocity[i,1]) for i in range(n_agents)]
plt.axis([-max_rectangle, max_rectangle, -max_rectangle, max_rectangle])

for i in range(n_time_Steps):
    px=[]
    py=[]
    for j in range(n_agents):
        robot = robots[j]
        robot.update_position(dt)
        px.append(robot.x)
        py.append(robot.y)
    plt.plot(px, py, 'ro')
    plt.pause(0.01)


    time += dt

plt.show()















