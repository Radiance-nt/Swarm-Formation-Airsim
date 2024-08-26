#!/usr/bin/python3
# coding:utf-8
import time
import rospy
from nav_msgs.msg import Odometry
import airsim

node = rospy.init_node('airsim_bridge_odom')
while not rospy.is_shutdown():
    if rospy.get_param('/reset', False):
        break

drone_id = rospy.get_param('~drone_id', default=-1)  # Default value is 0
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

client = airsim.MultirotorClient()
client.confirmConnection()
drones = client.listVehicles()
num_uavs = len(drones)

drone_name = drones[drone_id]
i = drone_id

odom_pub = rospy.Publisher('/drone_{}_visual_slam/odom'.format(i), Odometry, queue_size=10)

t = -1
while not rospy.is_shutdown():
    start_time = time.time()
    odom = client.simGetObjectPose(drones[i])

    drone_position = [odom.position.x_val, odom.position.y_val, odom.position.z_val]
    orientation = client.getMultirotorState(vehicle_name=drones[i]).kinematics_estimated.orientation

    x, y, z = odom.position.x_val, odom.position.y_val, odom.position.z_val
    x, y, z = y, x, -z + map_size[2] / 2
    odom.position.x_val, odom.position.y_val, odom.position.z_val = x, y, z
    # drone_orientation = [orientation.w_val, orientation.x_val, orientation.y_val,
    #                      orientation.z_val]
    #
    # r = R.from_euler('xyz', [90, 0, 0], degrees=True)
    # q = drone_orientation
    # rot_matrix = r.as_matrix()
    # rotated_vector = rot_matrix @ q[:3]
    # q_rotated = [rotated_vector[1], rotated_vector[2], rotated_vector[0], q[3]]
    # odom.orientation.w_val = q_rotated[3]
    # odom.orientation.x_val = q_rotated[0]
    # odom.orientation.y_val = q_rotated[1]
    # odom.orientation.z_val = q_rotated[2]

    odom_other = client.getMultirotorState(vehicle_name=drones[i]).kinematics_estimated
    # Odom to Odometry
    odom_msg = Odometry()
    odom_msg.header.stamp = rospy.Time.now()
    odom_msg.header.frame_id = 'world'
    odom_msg.child_frame_id = ''

    dx, dy, dz = odom_other.linear_velocity
    odom_msg.twist.twist.linear.x = dy
    odom_msg.twist.twist.linear.y = dx
    odom_msg.twist.twist.linear.z = -dz
    dx, dy, dz = odom_other.angular_velocity
    odom_msg.twist.twist.angular.x = dy
    odom_msg.twist.twist.angular.y = dx
    odom_msg.twist.twist.angular.z = -dz
    odom_msg.twist.twist.angular.z = -dz

    odom_msg.pose.pose.position.x = odom.position.x_val
    odom_msg.pose.pose.position.y = odom.position.y_val
    odom_msg.pose.pose.position.z = odom.position.z_val

    odom_msg.pose.pose.orientation.x = odom.orientation.x_val
    odom_msg.pose.pose.orientation.y = odom.orientation.y_val
    odom_msg.pose.pose.orientation.z = odom.orientation.z_val
    odom_msg.pose.pose.orientation.w = odom.orientation.w_val
    odom_pub.publish(odom_msg)

    t += 1
