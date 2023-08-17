import time
import numpy as np
from math import *
import scipy.linalg as la


class LqrUtil:
    def __init__(self, t, l):
        # 控制周期
        self.T = t
        # 车长
        self.L = l
        self.A = np.mat(np.eye(3))
        self.B = np.mat(np.zeros((3, 2)))
        # self.Q = np.mat(20*np.eye(3))
        self.Q = [[10, 0, 0], [0, 10, 0], [0, 0, 20]]
        self.R = np.mat(100*np.eye(2))
        # x,y,phi,vel,delta
        self.ref_data = [0, 0, 0, 0, 0]
        self.cur_data = [0, 0, 0, 0, 0]

    def set_cur_data(self, a, b, c):
        self.cur_data[0] = a
        self.cur_data[1] = b
        self.cur_data[2] = c

    def set_ref_data(self, x, y, yaw, delta, v):
        self.ref_data[0] = x
        self.ref_data[1] = y
        self.ref_data[2] = yaw
        self.ref_data[3] = v
        self.ref_data[4] = delta

    def set_state_matrix(self):
        T = self.T
        L = self.L
        phi = self.ref_data[2]
        vel = self.ref_data[3]
        delta = self.ref_data[4]
        self.A[0, 2] = -vel*sin(phi)*T
        self.A[1, 2] = vel*cos(phi)*T
        self.B[0, 0] = cos(phi)*T
        self.B[1, 0] = sin(phi)*T
        self.B[2, 0] = tan(delta)*T/L
        N = vel*T
        M = L*pow(cos(delta), 2)
        self.B[2, 1] = N/M

    def car_lqr(self):
        K = self.dlqr()
        e0 = self.cur_data[0] - self.ref_data[0]
        e1 = self.cur_data[1] - self.ref_data[1]
        e2 = self.cur_data[2] - self.ref_data[2]
        e2 = self.yaw_p2p(e2)
        E = [[e0], [e1], [e2]]
        car_vel = self.ref_data[3]-K[0, :]*E
        self.lf_vel = car_vel
        car_steer = self.ref_data[4]-K[1, :]*E
        return car_vel[0, 0], car_steer[0, 0]

    def dlqr(self):
        start_time = time.time()
        A = self.A
        B = self.B
        Q = self.Q
        R = self.R
        X = Q
        maxiter = 300
        eps = 0.01
        for i in range(maxiter):
            Xn = A.T * X * A - A.T * X * B * la.pinv(R + B.T * X * B) * B.T * X * A + Q
            if (abs(Xn - X)).max() < eps:
                break
            X = Xn
        K = la.pinv(B.T * X * B + R) * (B.T * X * A)
        end_time = time.time()
        print("lqr costs time：%.2fs" % (end_time - start_time))
        return K

    def yaw_p2p(self, e):
        if e > pi/2:
            e -= 2*pi
        elif e < -pi/2:
            e += 2*pi
        else:
            e = e
        return e
