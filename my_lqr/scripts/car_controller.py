#!/usr/bin/env python3.6
# -*- coding: utf-8 -*-

from lqr_util import LqrUtil
from car_obj import CarObj
from path_process import *
from lqr_debug import LqrDebug
import rospy
from my_lqr.msg import car_info
from my_lqr.msg import my_marker
from my_lqr.msg import way_points
from geometry_msgs.msg import Twist


def car_info_cb(data):
    my_car.set_position(data.car_x, data.car_y, data.car_yaw)
    my_car.set_cur_vel(data.car_v)


start_flag = False
ref_path = None
path_xs = None
path_ys = None
def global_path_cb(path_msg):
    global ref_path
    ref_path = path_msg.way_points
    global path_xs, path_ys
    path_xs, path_ys = ref_path2xy(ref_path)
    global start_flag
    start_flag = True


def car_controller(event):
    if not start_flag:
        print("wait for start")
        return
    # 控制 ********************************************************
    car_x, car_y, car_yaw = my_car.x, my_car.y, my_car.yaw
    lu.set_cur_data(car_x, car_y, car_yaw)
    index = find_forward_point(car_x, car_y, path_xs, path_ys)
    match_point = ref_path[index]
    ref_x, ref_y, ref_yaw, ref_steer = \
        match_point.x, match_point.y, match_point.yaw, match_point.delta
    ept_vel, k = my_car.get_expect_vel_k()
    lu.set_ref_data(ref_x, ref_y, ref_yaw, ref_steer, ept_vel)
    lu.set_state_matrix()
    car_vel, car_steer = lu.car_lqr()
    my_car.set_steer(car_steer)
    # debug ********************************************************
    ld.car_steer, ld.car_vel = car_steer, car_vel
    ld.ref_steer, ld.ref_yaw = lu.ref_data[4], lu.ref_data[2]
    ld.car_yaw = lu.cur_data[2]
    ld.print_info()
    # **************************************************************
    car_steer = car_steer*k
    msg = Twist()
    msg.linear.x = car_vel
    msg.angular.z = car_steer
    twist_pub.publish(msg)
    # marker.car_x = car_x
    # marker.car_y = car_y
    # marker.ref_x = lu.ref_data[0]
    # marker.ref_y = lu.ref_data[1]
    # marker_pub.publish(marker)


if __name__ == "__main__":
    print("start")
    car_l = 0.26
    T = 0.15
    straight_vel = 0.8
    arc_vel = 0.3
    my_car = CarObj(car_l)
    lu = LqrUtil(T, my_car.L)
    ld = LqrDebug()
    marker = my_marker()
    marker_pub = rospy.Publisher('/my_marker', my_marker, queue_size=1)
    my_car.set_expect_vel(straight_vel, arc_vel)
    rospy.init_node("car_controller", anonymous=True)
    rospy.Subscriber("/car_info", car_info, car_info_cb)
    rospy.Subscriber('/ref_path', way_points, global_path_cb)
    twist_string = '/cmd_vel'
    twist_pub = rospy.Publisher(twist_string, Twist, queue_size=1)
    timer = rospy.Timer(rospy.Duration(T), car_controller)
    rospy.spin()

