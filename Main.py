from Environment import Environment
from Robot import Robot
import numpy as np
import random
import math
import matplotlib.pyplot as plt

max_rectangle = 100
min_rectangle = 0
n_landmarks = 5
n_agents = 5
n_waypoints = 2
v_max = 10
total_time = 10
n_time_Steps = 1000
communication = True

ultrasonic_distance = 20

corners = [[min_rectangle, min_rectangle],
           [min_rectangle, max_rectangle],
           [max_rectangle, min_rectangle],
           [max_rectangle, max_rectangle]]
corners = np.array(corners)


start_points = np.array([[random.random(), random.random()] for i in range(n_agents)])*max_rectangle + 1
initial_u = (np.array([[random.random()*v_max, random.random()*2*3.14] for i in range(n_agents)]))

landmarks = np.array([[random.random(), random.random()] for i in range(n_landmarks)])*max_rectangle

dt = total_time/n_time_Steps
time = 0
robots = [Robot(x=start_points[i,0], y=start_points[i,1], v=initial_u[i,0], theta=initial_u[i,1], n_agents=n_agents, id=i)
          for i in range(n_agents)]
# plt.axis([-max_rectangle, max_rectangle, -max_rectangle, max_rectangle])
robot_true_positions = []
robots_estimated_positions = []
robots_error = []
time_track = []
for i in range(n_time_Steps):
    true_pos = []
    est_pos = []
    err_pos = []
    for j in range(n_agents):
        robot = robots[j]
        robot.update_true_position(dt)
        robot.update_odometer(v=robot.v, omega=robot.omega)
        robot.motion_update_landmark(dt)
        robot.sensor_update_landmark(landmarks)
        if communication:
            for k in range(n_agents):
                if j != k:
                    robot_k = robots[k]
                    Xj = np.array([robot_k.x_correct, robot_k.y_correct, robot_k.theta_correct])
                    noise = (random.random() * 2 - 1) * 0.1
                    d = math.sqrt((robot_k.x_truth - robot.x_truth) ** 2 + (robot_k.y_truth - robot.y_truth) ** 2) + noise
                    noise = (random.random() * 2 - 1) * 0.1
                    bearing = math.atan2(robot_k.y_truth - robot.y_truth,
                                         robot_k.x_truth - robot.x_truth) - robot.theta_truth + noise
                    bearing = robot.constraint_bearing(bearing)
                    readings= np.array([d, bearing])
                    robot.relative_measurement_update(Xj=Xj, sigma_ji=robot_k.sigma_ij[j], robot_id=robot_k.id, z_org=readings)
        true_pos.append([robot.x_truth, robot.y_truth, robot.theta_truth])
        est_pos.append([robot.x_correct, robot.y_correct, robot.theta_correct])
        err_pos.append([robot.x_truth - robot.x_correct, robot.y_truth - robot.y_correct, robot.theta_truth - robot.theta_correct])
        bearing = robot.sense_bearing()

    # plt.plot(px, py, 'ro')
    # plt.pause(0.01)

    time += dt
    time_track.append(time_track)
    robot_true_positions.append(true_pos)
    robots_estimated_positions.append(est_pos)
    robots_error.append(err_pos)
    print(time)
print('Done')

robot_true_positions = np.array(robot_true_positions)
robots_estimated_positions = np.array(robots_estimated_positions)
robots_error = np.array(robots_error)
time_track = np.array(time_track)
# plt.plot(time_track, robots_error[:,0], 'ro')
# plt.show()
gt = 1



# plt.show()















