#!/usr/bin/python3
import threading
import time
import os
import numpy as np
import random
import binvox_rw
from voxel import MapServer
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32, Float64, Bool
import airsim

rospy.init_node('control_variable_experiment')

seed = rospy.get_param('~seed', 0)
map_size_x = rospy.get_param('/map_size_x', 150)  # 80 available
map_size_y = rospy.get_param('/map_size_y', 150)  # 50 available
map_size_z = rospy.get_param('/map_size_z', 50)  # 50 available
ue_map_name = rospy.get_param('~map_name', "control")  # 50 available

map_size = (map_size_x, map_size_y, map_size_z)

random.seed(seed)
np.random.seed(seed)
rospy.set_param('/general/seed', seed)

while not rospy.is_shutdown():
    if rospy.get_param('/reset', False):
        break
    time.sleep(0.05)

client = airsim.MultirotorClient()
client.confirmConnection()
drones = client.listVehicles()
num_uavs = len(drones)

goal_reached = [False for i in range(num_uavs)]


def goal_reached_callback(msg, drone_id):
    global goal_reached
    # print(f"\n\n[Nav] {drone_id} reached !!\n\n")
    goal_reached[drone_id] = msg.data


[rospy.Subscriber(f"/drone_{i}_planning/finish", Bool, lambda msg, i=i: goal_reached_callback(msg, i)) for i in
 range(num_uavs)]


def _listener():
    rospy.spin()


thread = threading.Thread(target=_listener)
thread.start()

global_objects = client.simListSceneObjects()

for i in range(5):
    if f"Cylinder_{i}" in global_objects:
        client.simDestroyObject(f"Cylinder_{i}")

default_position = airsim.Vector3r()
default_orientation = airsim.Quaternionr()

map_center = (0, 0, 0)
res = 1
map_name = f"airsim_map_{ue_map_name}_{map_size_x}_{map_size_y}_{map_size_z}_{res}"
base_path = "/tmp"
map_path = os.path.join(base_path, map_name)
map_path_binvox = map_path + ".binvox"
if not os.path.exists(map_path_binvox):
    print("[Nav]: Generating Map Binvox Model From Airsim..")
    client.simCreateVoxelGrid(airsim.Vector3r(*map_center),
                              int(map_size[0]), int(map_size[1]), int(map_size[2]),
                              res,
                              map_path_binvox)
else:
    print("[Nav]: Loading Local Map Binvox From ", map_path_binvox)
print("[Nav]: Reading binvox model..")
with open(map_path_binvox, 'rb') as f:
    model = binvox_rw.read_as_3d_array(f)
print("[Nav]: Initializing map server..")

map_server = MapServer(model, map_center, res, inflation_radius=0, voxel_size=0, visualize_once=False,
                       map_size=map_size)

map_points = np.array(map_server.point_cloud.points)
np.save(map_path, map_points)
time.sleep(5)
rospy.set_param('/map_npy_path', map_path + ".npy")

max_values = np.max(map_points, axis=0)
min_values = np.min(map_points, axis=0)

grid_compress_scale = np.array([4, 4, 2])
safety_height = 5
safety_margin = 10

expected_deviation = 2
goal_publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
survivor_found_publisher = rospy.Publisher('/search/survivor_found', Int32, queue_size=10)
percentage_publisher = rospy.Publisher('/search/percentage', Float64, queue_size=10)

relative_positions = np.zeros((num_uavs, 3))
for i in range(num_uavs):
    for j, dim in enumerate(['x', 'y', 'z']):
        key = f'/drone_0_ego_planner_node/global_goal/relative_pos_{i}/{dim}'
        value = rospy.get_param(key)
        relative_positions[i, j] = value


def publish(goal_point, current_center=None):
    # Create a PoseStamped message
    goal_msg = PoseStamped()

    # Fill in the necessary fields of the message
    goal_msg.header.stamp = rospy.Time.now()
    goal_msg.header.frame_id = 'map'  # Assuming the frame is 'map', change if necessary
    goal_msg.pose.position.x = goal_point[1]
    goal_msg.pose.position.y = goal_point[0]
    goal_msg.pose.position.z = -goal_point[2] + map_size[2] / 2
    time.sleep(0.5)
    goal_publisher.publish(goal_msg)


while not rospy.is_shutdown():
    if rospy.get_param('/start_navigation', False):
        print("[Nav] Start navigation")
        time.sleep(3)
        break

nav_id = 0

# available_points = [(0, 27, 0), (40, 0, 0), (0, 0, 0), (40, 25, 0)]
available_points = [(40, 0, 0)]
available_points = [map_server.coord_to_index(point) for point in available_points]
nav_iter = iter(available_points)
success_nav = 0
success_nav_param_name = '/general/success_nav'
rospy.set_param(success_nav_param_name, success_nav)

threads = []
for i, drone_name in enumerate(drones):
    client.enableApiControl(True, vehicle_name=drone_name)
    client.armDisarm(True, vehicle_name=drone_name)

threads = []
for i, drone_name in enumerate(drones):
    th = client.moveByVelocityAsync(0, 0, -4, 1, vehicle_name=drone_name)
    threads.append(th)
for thread in threads:
    thread.join()

rospy.set_param('/takeoff', True)
rospy.set_param('/record/start_record', True)

time.sleep(1)

while not rospy.is_shutdown():
    positions = [client.simGetObjectPose(drones[i]).position for i in range(num_uavs)]
    positions = [p.to_numpy_array() for p in positions]
    positions = np.stack(positions)

    track_points = np.zeros_like(positions)
    real_world_paths = []
    nav_grid_index = next(nav_iter)
    # end_map = map_server.coord_to_index(nav_grid_index)
    end_map = nav_grid_index
    mean_position = positions.mean(0)
    start_map = map_server.coord_to_index(mean_position)
    threshold = 20
    path = [end_map]
    # map_server.visualize_path(path, start_map, end_map)
    real_world_path = [map_server.index_to_coord(point) for point in path]
    if len(real_world_path) == 0:
        map_server.visualize_path(path, start_map, end_map)
    for i, drone_name in enumerate(drones):
        real_world_paths.append(real_world_path)
        track_points[i] = real_world_path[0]
    print("[Nav]: Get the plan", real_world_paths)

    shared_path_iter = iter(real_world_path)

    t = 0
    dt = 0.1
    threads = []
    track = np.array(next(shared_path_iter))
    publish(track, mean_position)
    rospy.set_param(f"/general/nav_point_{success_nav}",
                    {'x': available_points[success_nav][0],
                     'y': available_points[success_nav][1],
                     'z': available_points[success_nav][2]})

    while not rospy.is_shutdown():

        positions = [client.simGetObjectPose(drones[i]).position
                     for i in range(num_uavs)]
        positions = [p.to_numpy_array() for p in positions]
        positions = np.stack(positions)
        mean_position = positions.mean(0)
        relative_positions.mean(0)
        # swarm_all_reached = np.linalg.norm(track + relative_positions.mean(0) - mean_position) < expected_deviation
        swarm_all_reached = all(goal_reached)
        if swarm_all_reached:
            goal_reached = [False for i in range(num_uavs)]
            try:
                track = np.array(next(shared_path_iter))
            except:
                time.sleep(2)
                success_nav += 1
                rospy.set_param(success_nav_param_name, success_nav)
                break

            publish(track, mean_position)
            rospy.set_param(f"/general/nav_point_{success_nav}",
                            {'x': available_points[success_nav][0],
                             'y': available_points[success_nav][1],
                             'z': available_points[success_nav][2]})

        t += dt
        time.sleep(0.1)

while not rospy.is_shutdown():
    time.sleep(0.1)
