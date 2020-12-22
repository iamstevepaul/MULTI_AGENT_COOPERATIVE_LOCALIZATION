from EKF import EKF
import numpy as np
import math
from random import random


class Robot:

    def __init__(self,
                 x=0.0,
                 y=0.0,
                 v=0.0,
                 theta=0.0,
                 omega = 0.0,
                 ultra_sonic=0.0,
                 bearing=0.0,
                 gps=0.0,
                 n_agents = 10,
                 id = 0,
                 sensing_range = 20,
                 v_noise = 0.1,
                 omega_noise = 0.1,
                 range_noise = 0.1,
                 bearing_noise = 0.1):
        self.x_truth = x
        self.y_truth = y
        self.x_predict = x
        self.x_correct = x
        self.y_predict = y
        self.y_correct = y
        self.v = v # odometer reading
        self.theta_truth = theta # bearing angle
        self.theta_predict = theta
        self.theta_correct = theta
        self.omega = omega
        self.Sigma_ii = 0.1*np.identity(3) # number of states is 3 - this is the covariance matrix
        self.Sigma_ij = [0.1 * np.identity(3) for i in range(n_agents)]
        self.sigma_ij = [0.1*np.identity(3) for i in range(n_agents)]
        self.ultra_sonic = ultra_sonic
        self.bearing = bearing
        self.gps = gps
        self.alpha1 = 0.1
        self.alpha2 = 0.03
        self.alpha3 = 0.09
        self.alpha4 = 0.08
        self.id = id
        self.n_agents = n_agents
        self.sensing_range = sensing_range

        ## noises
        self.v_noise = v_noise
        self.omega_noise = omega_noise
        self.range_noise = range_noise
        self.bearing_noise = bearing_noise

    def update_true_position(self, dt): # this should be updated
        self.theta_truth += self.omega*dt
        self.theta_truth = self.constraint_bearing(self.theta_truth)
        self.x_truth += self.v * dt*math.cos(self.theta_truth)
        self.y_truth += self.v * dt*math.sin(self.theta_truth)
        # theta_truth does not change

    def sense_odometry(self):
        noise = np.random.normal(0, self.v_noise, 1)[0]
        v = self.v + noise
        noise = np.random.normal(0, self.omega_noise, 1)[0]
        omega = self.omega + noise
        return np.array([v, omega])

    def motion_update_landmark(self, dt):
        #update this to update the cross correlation --  done
        # this method should only be called after odometery update
        odo_vals = self.sense_odometry()
        v = odo_vals[0]
        omega = odo_vals[1]
        self.x_predict = self.x_correct + v*dt*math.cos(self.theta_correct + omega*dt/2)
        self.y_predict = self.y_correct + v * dt * math.sin(self.theta_correct + omega * dt / 2)
        self.theta_predict = self.theta_correct + omega*dt

        # update covariance
        Gt = np.array([[1, 0, -v*dt*math.sin(self.theta_correct + omega * dt)],
                       [0, 1, v*dt*math.cos(self.theta_correct + omega * dt)],
                       [0, 0, 1]])

        Mt = np.array([[self.alpha1*(v*v) + self.alpha2*(omega*omega), 0],
                       [0, self.alpha3*(v*v) + self.alpha4*(omega*omega)]])
        Vt = np.array([[math.cos(self.theta_correct + omega*dt), -math.sin(self.theta_correct + omega*dt)],
                       [math.sin(self.theta_correct + omega*dt), math.cos(self.theta_correct + omega*dt)],
                       [0, 1]])
        self.Sigma_ii = np.matmul(np.matmul(Gt, self.Sigma_ii), np.transpose(Gt)) + \
                     np.matmul(np.matmul(Vt, Mt), np.transpose(Vt))
        for j in range(self.n_agents):
            if j != self.id:
                self.Sigma_ij[j] = np.matmul(Gt,self.Sigma_ij[j])
                self.sigma_ij[j] = np.matmul(Gt, self.sigma_ij[j])
        # print(self.sigma_ii
        # )

    def sense_ultra_sonic_landmark(self, landmarks):

        #change the noise here
        readings = []
        for landmark in landmarks:
            noise = (np.random.normal(0, self.range_noise, 1)[0])
            d = math.sqrt((landmark[0] - self.x_truth)**2 + (landmark[1] - self.y_truth)**2) + noise
            noise = (np.random.normal(0, self.bearing_noise, 1)[0])
            bearing = math.atan2(landmark[1] - self.y_truth, landmark[0] - self.x_truth) - self.theta_truth + noise
            bearing = self.constraint_bearing(bearing)
            readings.append([d, bearing])
        return np.array(readings)

    @staticmethod
    def constraint_bearing(bearing):
        while bearing < 0:
            bearing = bearing + 2 * math.pi

        while bearing > 2*math.pi:
            bearing = bearing - 2 * math.pi
        return bearing

    def sensor_update_landmark(self, landmarks):
        # update this to update the cross correlation - done
        z_org = self.sense_ultra_sonic_landmark(landmarks)
        i=0
        for landmark in landmarks:
            x_dist = landmark[0] - self.x_predict
            y_dist = landmark[1] - self.y_predict
            q = x_dist**2 + y_dist**2

            if np.sqrt((landmark[0] - self.x_truth)**2 + (landmark[1] - self.y_truth)**2) <= self.sensing_range:

                bearing = math.atan2(y_dist, x_dist) - self.theta_predict
                bearing = self.constraint_bearing(bearing)
                z_hat_i = np.array([math.sqrt(q), bearing])
                Ht_i = np.array([[-x_dist/math.sqrt(q), -y_dist/math.sqrt(q), 0], [y_dist/q, -x_dist/q, -1]])
                sigma_range = 2
                sigma_bearing = 3
                Qt = [[sigma_range**2, 0], [0, sigma_bearing**2]]
                St = np.matmul(np.matmul(Ht_i, self.Sigma_ii), np.transpose(Ht_i)) + Qt  # add the Qt matrix as well
                Kt_i = np.matmul(np.matmul(self.Sigma_ii, np.transpose(Ht_i)), np.linalg.inv(St))
                corr = np.matmul(Kt_i, z_org[i]-z_hat_i)

                self.x_predict += corr[0]
                self.y_predict += corr[1]
                self.theta_predict += corr[2]
                self.theta_predict = self.constraint_bearing(self.theta_predict)
                self.Sigma_ii = np.matmul(np.identity(3) - np.matmul(Kt_i, Ht_i), self.Sigma_ii)
                for j in range(self.n_agents):
                    if j != self.id:
                        self.Sigma_ij[j] = np.matmul(np.identity(3) - np.matmul(Kt_i, Ht_i), self.Sigma_ij[j])
                        self.sigma_ij[j] = np.matmul(np.identity(3) - np.matmul(Kt_i, Ht_i), self.sigma_ij[j])
            i=i+1

        self.x_correct = self.x_predict
        self.y_correct = self.y_predict
        self.theta_correct = self.theta_predict
        # print(self.x_truth, self.x_correct)
        # print(self.y_truth, self.y_correct)
        # print('Theta')
        # print(self.theta_truth, self.theta_correct)


    def relative_measurement_update(self, Xj, sigma_ji, z_org, robot_id):
        # think of including equations 12, 13, and 14 properly

        self.Sigma_ij[robot_id] = np.matmul(self.sigma_ij[robot_id], np.transpose(sigma_ji))
        xj = Xj[0]
        yj = Xj[1]
        theta_j = Xj[2]
        (xj - self.x_correct) ** 2 + (yj - self.y_correct) ** 2
        q = (xj - self.x_correct)**2 + (yj - self.y_correct)**2
        Ht = np.array([[-(xj - self.x_correct)/math.sqrt(q), -(yj - self.y_correct)/math.sqrt(q), 0],
                       [(yj - self.y_correct)/q, (xj - self.x_correct)/q, -1]])
        Ht_j = np.array([[(xj - self.x_correct)/math.sqrt(q), (yj - self.y_correct)/math.sqrt(q), 0],
                       [-(yj - self.y_correct)/q, -(xj - self.x_correct)/q, -1]])
        sigma_range = 2
        sigma_bearing = 3
        Qt = [[sigma_range ** 2, 0], [0, sigma_bearing ** 2]]
        St = np.matmul(np.matmul(Ht, self.Sigma_ij[robot_id]), np.transpose(Ht)) + Qt
        Kt_i = np.matmul(np.matmul(self.Sigma_ii, np.transpose(Ht)) + np.matmul(self.Sigma_ij[robot_id], np.transpose(Ht)) , np.linalg.inv(St))
        bearing = math.atan2((yj - self.y_correct), (xj - self.x_correct)) - self.theta_predict
        bearing = self.constraint_bearing(bearing)
        z_hat_i = np.array([math.sqrt(q), bearing])
        corr = np.matmul(Kt_i, z_org - z_hat_i)
        self.x_predict += corr[0]
        self.y_predict += corr[1]
        self.theta_predict += corr[2]
        self.theta_predict = self.constraint_bearing(self.theta_predict)
        Sigma_ji = np.matmul(sigma_ji, np.transpose(self.sigma_ij[robot_id]))
        self.Sigma_ii = np.matmul(np.identity(3) - np.matmul(Kt_i, Ht), self.Sigma_ii)\
                        - np.matmul(np.matmul(Kt_i, Ht_j), Sigma_ji)
        self.Sigma_ij[robot_id] = np.matmul(np.identity(3) - np.matmul(Kt_i, Ht), self.Sigma_ij[robot_id])
        for j in range(self.n_agents):
            if j != self.id and j !=  robot_id:
                self.Sigma_ij[j] = np.matmul(np.identity(3) - np.matmul(Kt_i, Ht), self.Sigma_ij[j])
                self.sigma_ij[j] = np.matmul(np.identity(3) - np.matmul(Kt_i, Ht), self.sigma_ij[j])
        self.sigma_ij[robot_id] = self.Sigma_ij[robot_id]

    def sense_bearing(self):
        return self.bearing + (random()*2 -1) ## this has to be changed

    def sense_gps(self):
        pass

    def update_odometer(self, v,omega):
        self.v = v
        self.omega = omega