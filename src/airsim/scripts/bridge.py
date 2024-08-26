#!/usr/bin/python3
# coding:utf-8
import threading
import time
import airsim
import rospy
from airsim import Vector3r
from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import numpy as np
from quadrotor_msgs.msg import PositionCommand
from std_msgs.msg import Header
from sensor_msgs import point_cloud2

client = airsim.MultirotorClient()
client.confirmConnection()
client.reset()
client.simFlushPersistentMarkers()
rospy.set_param('/reset', True)

drones = client.listVehicles()
num_uavs = len(drones)

node = rospy.init_node('airsim_bridge')
rospy.set_param('/start_navigation', False)
rospy.set_param('/global/num_uavs', num_uavs)

map_points = None
while not rospy.is_shutdown():
    map_path = rospy.get_param('/map_npy_path', None)
    if map_path:
        map_points = np.load(map_path)
        map_size_x = rospy.get_param('/map_size_x')
        map_size_y = rospy.get_param('/map_size_y')
        map_size_z = rospy.get_param('/map_size_z')
        map_size = (map_size_x, map_size_y, map_size_z)
        print("[Bridge]: Map Size:", map_size)
        print("[Bridge]: load map from", map_path)
        break
    time.sleep(0.05)

assert map_points is not None
ros_map_points = map_points - np.array(map_size) / 2
ros_map_points[:, 2] += map_size[2] / 2
map_pub = rospy.Publisher('/map_generator/global_cloud', PointCloud2, queue_size=10)

header = Header()
header.stamp = rospy.Time.now()
header.frame_id = 'world'
rospy.set_param('/start_navigation', True)

t = -1
initial_orientations = []
while not rospy.is_shutdown():
    if rospy.get_param("/takeoff", False) and len(initial_orientations) == 0:
        for i, drone_name in enumerate(drones):
            odom = client.simGetObjectPose(drones[i])

            drone_position = [odom.position.x_val, odom.position.y_val, odom.position.z_val]
            drone_orientation = [odom.orientation.w_val, odom.orientation.x_val, odom.orientation.y_val,
                                 odom.orientation.z_val]
            initial_orientations.append(drone_orientation)

    t += 1
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = 'world'

    positions = [client.simGetObjectPose(drones[i]).position
                 for i in range(num_uavs)]
    positions = [p.to_numpy_array() for p in positions]
    positions = np.stack(positions)
    ros_positions = positions
    ros_positions[:, 0], ros_positions[:, 1], ros_positions[:, 2] = (positions[:, 1],
                                                                     positions[:, 0],
                                                                     -positions[:, 2] + map_size[2] / 2)
    mean_position = ros_positions.mean(0)
    mask = np.all((ros_map_points >= mean_position - 30) & (ros_map_points <= mean_position + 30), axis=1)
    pub_point = ros_map_points[mask]
    pc_data = point_cloud2.create_cloud_xyz32(header, pub_point)
    map_pub.publish(pc_data)

for i, drone_name in enumerate(drones):
    client.enableApiControl(False, vehicle_name=drone_name)
    client.armDisarm(False, vehicle_name=drone_name)
