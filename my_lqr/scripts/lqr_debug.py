import math

class LqrDebug:
    def __init__(self):
        self.car_yaw = 0
        self.car_vel = 0
        self.ref_yaw = 0
        self.ref_steer = 0
        self.car_steer = 0

    def print_info(self):
        ref_steer = self.rad2deg(self.ref_steer)
        ref_yaw = self.rad2deg(self.ref_yaw)
        print("ref_steer:{:.2f},ref_yaw:{:.2f}".format(ref_steer, ref_yaw))
        car_steer = self.rad2deg(self.car_steer)
        car_yaw = self.rad2deg(self.car_yaw)
        print("car_steer:{:.2f},car_yaw:{:.2f}".format(car_steer, car_yaw))
        print("car_vel:{:.2f}".format(self.car_vel))

    def rad2deg(self, x):
        x = x / math.pi * 180
        return x