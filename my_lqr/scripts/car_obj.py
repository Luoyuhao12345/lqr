import math


class CarObj:
    def __init__(self, car_l):
        self.L = car_l
        self.steer = 0
        self.straight_vel = 0
        self.arc_vel = 0
        self.cur_vel = 0
        self.x = 0
        self.y = 0
        self.yaw = 0

    def set_cur_vel(self, x):
        self.cur_vel = x

    def set_expect_vel(self, x, y):
        self.straight_vel = x
        self.arc_vel = y

    def set_position(self, a, b, c):
        self.x = a
        self.y = b
        self.yaw = c

    def set_steer(self, x):
        self.steer = x

    def get_expect_vel_k(self):
        steer = self.steer
        steer = abs(steer * 180 / math.pi)
        if steer < 5:
            expect_v = self.straight_vel
            k = 1
        else:
            expect_v = self.arc_vel
            k = 1.5
        return expect_v, k
