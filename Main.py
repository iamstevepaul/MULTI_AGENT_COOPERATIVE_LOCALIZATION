from Environment import Environment
from Robot import Robot
import numpy as np
import random
import math
import matplotlib.pyplot as plt

max_rectangle = 100
min_rectangle = 0
n_landmarks = 6
n_agents = 7
n_waypoints = 2
v_max = 10
v_min = 10
total_time = 10
n_time_Steps = 1000
communication = True
sensing_range = 70


## noises
v_noise = 0.05
omega_noise = 0.05
range_noise = 0.05
bearing_noise = 0.05

corners = [[min_rectangle, min_rectangle],
           [min_rectangle, max_rectangle],
           [max_rectangle, min_rectangle],
           [max_rectangle, max_rectangle]]
corners = np.array(corners)


start_points = np.array([[random.random(), random.random()] for i in range(n_agents)])*max_rectangle + 1
initial_u = (np.array([[max(random.random()*v_max+v_min, v_max), random.random()*2*3.14] for i in range(n_agents)]))
omega = 0.628
landmarks = np.array([[random.random(), random.random()] for i in range(n_landmarks)])*max_rectangle

dt = total_time/n_time_Steps

time = 0
robots = [Robot(x=start_points[i,0],
                y=start_points[i,1],
                v=initial_u[i,0],
                theta=initial_u[i,1],
                n_agents=n_agents,
                id=i,
                omega = omega,
                v_noise=v_noise,
                omega_noise=omega_noise,
                range_noise=range_noise,
                bearing_noise=bearing_noise,
                sensing_range=sensing_range)
          for i in range(n_agents)]
robot_true_positions = []
robots_estimated_positions = []
robots_error = []
time_track = []
sigma = []
for i in range(n_time_Steps):
    sigma_i = []
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
                    if np.sqrt((robot.x_truth-robot_k.x_truth)**2 + (robot.y_truth - robot_k.y_truth)**2
                               < robot.sensing_range):
                        Xj = np.array([robot_k.x_correct, robot_k.y_correct, robot_k.theta_correct])
                        noise = (np.random.normal(0, robot.range_noise, 1)[0])
                        d = math.sqrt((robot_k.x_truth - robot.x_truth) ** 2
                                      + (robot_k.y_truth - robot.y_truth) ** 2) \
                            + noise
                        noise = (np.random.normal(0, robot.bearing_noise, 1)[0])
                        bearing = math.atan2(robot_k.y_truth - robot.y_truth,
                                             robot_k.x_truth - robot.x_truth) - robot.theta_truth + noise
                        bearing = robot.constraint_bearing(bearing)
                        readings= np.array([d, bearing])
                        robot.relative_measurement_update(Xj=Xj,
                                                          sigma_ji=robot_k.sigma_ij[j],
                                                          robot_id=robot_k.id,
                                                          z_org=readings)
        sigma_i.append(np.diag(robot.Sigma_ii))
        true_pos.append([robot.x_truth, robot.y_truth, robot.theta_truth])
        est_pos.append([robot.x_correct, robot.y_correct, robot.theta_correct])
        err_pos.append([robot.x_truth - robot.x_correct,
                        robot.y_truth - robot.y_correct,
                        robot.theta_truth - robot.theta_correct])
        bearing = robot.sense_bearing()


    time += dt
    sigma.append(sigma_i)
    time_track.append(time_track)
    robot_true_positions.append(true_pos)
    robots_estimated_positions.append(est_pos)
    robots_error.append(err_pos)
    print(time)


robot_true_positions = np.array(robot_true_positions)
robots_estimated_positions = np.array(robots_estimated_positions)
sigma = np.array(sigma)
sigma30 = 3*sigma[:,0, 0]**.5
sigma31 = 3*sigma[:,0, 1]**.5
sigma32 = 3*sigma[:,0, 2]**.5
robots_error = np.array(robots_error)
robots_error_rmse = ((robots_error**2).mean(1))**0.5
# plt.plot(robots_error_rmse, label='Error')
# plt.show()

# time_track = np.array(time_track)

SMALL_SIZE = 16
MEDIUM_SIZE = 20
BIGGER_SIZE = 25

dt = 0

plt.rc('font', size=MEDIUM_SIZE)          # controls default text sizes
plt.rc('axes', titlesize=MEDIUM_SIZE)     # fontsize of the axes title
plt.rc('axes', labelsize=MEDIUM_SIZE)    # fontsize of the x and y labels
plt.rc('xtick', labelsize=MEDIUM_SIZE)    # fontsize of the tick labels
plt.rc('ytick', labelsize=MEDIUM_SIZE)    # fontsize of the tick labels
plt.rc('legend', fontsize=MEDIUM_SIZE)    # legend fontsize
plt.rc('figure', titlesize=BIGGER_SIZE)

plt.subplot(311)
plt.title('Estimation error with 3\u03C3 bounds')
plt.plot(sigma30, 'g-')
plt.plot(-sigma30, 'g-', label='3\u03C3')
plt.legend()
plt.ylabel('Error in X (meters)')
plt.plot(robots_error[:,0,0], label='Error')
plt.legend()
plt.subplot(312)
plt.plot(sigma31, 'g-')
plt.plot(-sigma31, 'g-')
plt.ylabel('Error in Y (meters)')
# plt.legend('3\u03C3 -3\u03C3 Error')
plt.plot(robots_error[:,0,1])
plt.subplot(313)
plt.plot(sigma32, 'g-')
plt.plot(-sigma32, 'g-')
plt.ylabel('Error in \u03B8 (radians)')
plt.plot(robots_error[:,0,2])
plt.xlabel('Time in milli seconds')
# plt.legend('3\u03C3 -3\u03C3 Error')
# plt.plot(list(robots_estimated_positions[:,0,0]), list(robots_estimated_positions[:,0,1]))
# plt.plot(list(robot_true_positions[:,0,0]), list(robot_true_positions[:,0,1]))
plt.show()



# plt.show()















