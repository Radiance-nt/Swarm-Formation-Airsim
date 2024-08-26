#!/usr/bin/python3
# coding:utf-8
import threading
import time
import numpy as np
import rospy
from quadrotor_msgs.msg import PositionCommand
import airsim
from airsim import Vector3r, VelocityControllerGains, PIDGains

node = rospy.init_node('airsim_bridge_cmd')
while not rospy.is_shutdown():
    if rospy.get_param('/reset', False):
        break

drone_id = rospy.get_param('~drone_id', default=-1)  # Default value is 0
map_size_x = rospy.get_param('/map_size_x', 150)  # 80 available
map_size_y = rospy.get_param('/map_size_y', 150)  # 50 available
map_size_z = rospy.get_param('/map_size_z', 50)  # 50 available
map_size = (map_size_x, map_size_y, map_size_z)
if drone_id < 0:
    raise Exception("drone id")

while not rospy.is_shutdown():
    if rospy.get_param('/reset', False):
        break
    time.sleep(0.05)

client2 = airsim.MultirotorClient()
client2.confirmConnection()
drones = client2.listVehicles()
num_uavs = len(drones)

drone_name = drones[drone_id]

dx, dy, dz = 0, 0, 0
dt = 0.01

default_config = {
    "k_p": 1.3,
    "k_i": 0.05,
    "k_d": 0.1,
    "integral_limit": 6,
}

config_data = default_config
k_p = config_data.get("k_p", default_config["k_p"])
k_i = config_data.get("k_i", default_config["k_i"])
k_d = config_data.get("k_d", default_config["k_d"])
integral_limit = config_data.get("integral_limit", default_config["integral_limit"])

rospy.set_param("k_p", k_p)
rospy.set_param("k_i", k_i)
rospy.set_param("k_d", k_d)

for i in range(3):
    rospy.set_param(f"/kp_pos_{i}", k_p)
    rospy.set_param(f"/ki_pos_{i}", k_i)
    rospy.set_param(f"/kd_pos_{i}", k_d)
    rospy.set_param(f"/integral_pos_max_{i}", integral_limit)


def tf_from_ros(values, pos=False):
    a, b, c = values
    if pos:
        return b, a, -(c - map_size[2] / 2)
    return b, a, -c


client2.setVelocityControllerGains(
    velocity_gains=VelocityControllerGains(
        x_gains=PIDGains(0.8, 0.0, 0.05),
        y_gains=PIDGains(0.8, 0.0, 0.05),
        z_gains=PIDGains(2.0, 2.0, 0)
    ),
    vehicle_name=drone_name)


def pos_cmd_callback(msg: PositionCommand):
    global dx, dy, dz
    desired_velocity = (msg.velocity.x, msg.velocity.y, msg.velocity.z)
    desired_velocity = tf_from_ros(desired_velocity)
    desired_velocity = np.array(desired_velocity)
    dx, dy, dz = desired_velocity
    client2.moveByVelocityAsync(
        dx,
        dy,
        dz,
        dt,
        vehicle_name=drone_name
    )


topic_name = f'/drone_{drone_id}_planning/pos_cmd'
subscriber = rospy.Subscriber(topic_name, PositionCommand, pos_cmd_callback, queue_size=10)
while not rospy.is_shutdown():
    rospy.spin()
