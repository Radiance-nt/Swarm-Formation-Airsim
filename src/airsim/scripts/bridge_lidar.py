#!/usr/bin/python3
# coding:utf-8
import numpy as np
import time
import rospy
import airsim
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2
from std_msgs.msg import Header

node = rospy.init_node('airsim_bridge_lidar')
while not rospy.is_shutdown():
    if rospy.get_param('/reset', False):
        break

drone_id = rospy.get_param('~drone_id', default=-1)
mapping_interval = rospy.get_param('~mapping_interval', default=0.5)

local_range_x = rospy.get_param('~local_range_x', default=10)
local_range_y = rospy.get_param('~local_range_y', default=10)
local_range_z = rospy.get_param('~local_range_z', default=10)

map_size_x = rospy.get_param('/map_size_x')
map_size_y = rospy.get_param('/map_size_y')
map_size_z = rospy.get_param('/map_size_z')
map_size = (map_size_x, map_size_y, map_size_z)
if drone_id < 0:
    raise Exception("drone id")

while not rospy.is_shutdown():
    if rospy.get_param('/reset', False):
        break
    time.sleep(0.05)

map_points = None
while not rospy.is_shutdown():
    map_path = rospy.get_param('/map_npy_path', None)
    if map_path:
        map_points = np.load(map_path)
        break
    time.sleep(0.05)

assert map_points is not None
ros_map_points = map_points - np.array(map_size) / 2
ros_map_points[:, 2] += map_size[2] / 2

client = airsim.MultirotorClient()
client.confirmConnection()
drones = client.listVehicles()
num_uavs = len(drones)

positions = [client.simGetObjectPose(drones[i]).position
             for i in range(num_uavs)]
positions = [p.to_numpy_array() for p in positions]
positions = np.stack(positions)
positions[:, 0], positions[:, 1], positions[:, 2] = positions[:, 1], positions[:, 0], -positions[:, 2] + map_size[
    2] / 2
initial_positions = positions

drone_name = drones[drone_id]
i = drone_id

lidar_pub = rospy.Publisher('/drone_{}_pcl_render_node/cloud'.format(i), PointCloud2, queue_size=10)
header = Header()
header.stamp = rospy.Time.now()
header.frame_id = 'world'

t = -1

USING_MAP = True


def parse_lidarData(data):
    # reshape array of floats to array of [X,Y,Z]
    points = np.array(data.point_cloud, dtype=np.dtype('f4'))
    points = np.reshape(points, (int(points.shape[0] / 3), 3))

    return points


while not rospy.is_shutdown():
    start_time = time.time()

    if USING_MAP:
        window_size = max([local_range_x, local_range_y, local_range_z])

        odom = client.simGetObjectPose(drones[i])
        x, y, z = odom.position.x_val, odom.position.y_val, odom.position.z_val
        x, y, z = y, x, -z + map_size[2] / 2
        odom.position.x_val, odom.position.y_val, odom.position.z_val = x, y, z

        differences = ros_map_points - np.array([x, y, z])
        absolute_differences = np.abs(differences)
        within_window = np.all(absolute_differences <= window_size, axis=1)
        lidar_points = ros_map_points[within_window]

        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'world'
        header.seq = t
        lidar_msg = point_cloud2.create_cloud_xyz32(header, lidar_points)
    else:
        lidar_data = client.getLidarData(vehicle_name=drones[i])
        # convert lidar_data to PointCloud2 and publish
        lidar_points = parse_lidarData(lidar_data)
        lidar_points = lidar_points + initial_positions[i]

        lidar_points_x, lidar_points_y, lidar_points_z = lidar_points[:, 0], lidar_points[:, 1], lidar_points[:,
                                                                                                 2]
        lidar_points_x, lidar_points_y, lidar_points_z = lidar_points_y, lidar_points_x, -lidar_points_z
        lidar_points[:, 0], lidar_points[:, 1], lidar_points[:,
                                                2] = lidar_points_x, lidar_points_y, lidar_points_z

        lidar_msg = PointCloud2()
        lidar_msg.header.stamp = rospy.Time.now()
        lidar_msg.header.frame_id = 'map'
        lidar_msg.header.seq = t

        lidar_msg.height = 0
        lidar_msg.width = len(lidar_points)
        lidar_msg.fields = [PointField('x', 0, PointField.FLOAT32, 1),
                            PointField('y', 4, PointField.FLOAT32, 1),
                            PointField('z', 8, PointField.FLOAT32, 1)]
        lidar_msg.is_bigendian = False
        lidar_msg.point_step = 12
        lidar_msg.row_step = lidar_msg.point_step * lidar_msg.width
        lidar_msg.is_dense = True
        lidar_msg.data = np.asarray(lidar_points, np.float32).tostring()
    lidar_pub.publish(lidar_msg)

    t += 1
    time.sleep(mapping_interval)
