#!/usr/bin/python3
import os
import threading
from copy import deepcopy
import cv2
import numpy as np
import time
import rospy
from std_msgs.msg import Int8
import airsim

rospy.init_node('video_recorder')

fps = rospy.get_param('~fps', 30)
accelerate_rate = rospy.get_param('~video_accelerate', 1)
video_prefix = rospy.get_param('~video_prefix', "airsim_video")
camera_name = rospy.get_param('~camera_name', "Fixed")
camera_k = rospy.get_param('~camera_k', 1.2)
camera_b = rospy.get_param('~camera_b', 10)
camera_alpha = rospy.get_param('~camera_ma_alpha', 0.5)

virtual_speedup = rospy.get_param('/record/virtual_speedup', 1)
accelerate_rate *= virtual_speedup
sample_interval = 1 / fps * accelerate_rate

while not rospy.is_shutdown():
    if rospy.get_param('/reset', False):
        break

client = airsim.MultirotorClient()
client.confirmConnection()
drones = client.listVehicles()

objects = client.simListSceneObjects()
if f"external_{camera_name}" not in objects:
    rospy.logerr(f"[Rec]: Camera {camera_name} does not exist!")
    exit()

vehicle_name = ""
external = True
frame_size = (1080, 720) if "low_res" not in camera_name else (540, 360)
# frame_size = (540, 360)
fourcc = cv2.VideoWriter_fourcc(*'XVID')

# 初始化视频录制相关变量
video_writer = None
i = -1

while not rospy.is_shutdown():
    if rospy.get_param('/record/start_record', False):
        break

num_uavs = len(drones)

dir_name = ""
while not rospy.is_shutdown():
    dir_name = rospy.get_param('/record/dir_name', None)
    rospy.loginfo(f"[Video] Record Video Dir {dir_name}...")
    if dir_name:
        break
    rospy.sleep(0.1)

msg_data = i


def section_callback(msg):
    global i, video_writer, msg_data
    msg_data = msg.data


rospy.Subscriber("/record/section", Int8, section_callback)


def _listener():
    rospy.spin()


rospy.loginfo(f"[Video] Start listener...")

thread = threading.Thread(target=_listener)
thread.start()
video_filename = ""

ema_camera_position = None
alpha = camera_alpha

# 循环处理视频帧
frame_count = 0
start_time = time.time()
last_frame_time = time.time()
while not rospy.is_shutdown():
    current_time = time.time()
    if msg_data != i:

        # 如果有新的部分，结束当前视频并开始新的视频录制
        if video_writer is not None and video_writer.isOpened():
            video_writer.release()
            rospy.loginfo(f"[Video] Video saved to {os.path.join(dir_name, f'{video_prefix}_{i}.avi')}")

        i = msg_data
        video_filename = os.path.join(dir_name, f'{video_prefix}_{i}.avi')
        video_writer = cv2.VideoWriter(video_filename, fourcc, fps, frame_size)
        rospy.loginfo(f"[Video] Start recording {video_filename}")

    if not rospy.get_param('/record/start_record', True) and video_writer is not None and video_writer.isOpened():
        video_writer.release()
        rospy.loginfo(f"[Video] Video saved to {os.path.join(dir_name, f'{video_prefix}_{i}.avi')}")
        rospy.loginfo(f"[Video] Done")
        break

    expected_frames = int((current_time - start_time) / sample_interval)
    frames_to_write = expected_frames - frame_count

    if video_writer is not None and frames_to_write > 0:
        positions = [client.simGetObjectPose(drones[i]).position
                     for i in range(num_uavs)]
        positions = [p.to_numpy_array() for p in positions]
        positions = np.stack(positions)
        mean_position = positions.mean(0)
        camera_position = deepcopy(mean_position)
        distances = np.sqrt(((positions - mean_position) ** 2).sum(axis=1))
        max_distance = np.max(distances)
        camera_position[2] = mean_position[2] - max_distance * camera_k - camera_b

        if ema_camera_position is None:
            ema_camera_position = camera_position
        else:
            ema_camera_position = alpha * camera_position + (1 - alpha) * ema_camera_position

        camera_pose = airsim.Pose(airsim.Vector3r(*ema_camera_position.tolist()),
                                  airsim.to_quaternion(-1.5707, 0, 0))

        client.simSetCameraPose(camera_name, camera_pose, external=external)

        frame_start_time = time.time()  # 开始处理帧的时间

        response = client.simGetImage(camera_name, airsim.ImageType.Scene, vehicle_name=vehicle_name, external=external)

        if response:
            np_response = np.frombuffer(response, dtype=np.uint8)
            frame = cv2.imdecode(np_response, cv2.IMREAD_COLOR)
            # frame = cv2.resize(frame, frame_size)
            for _ in range(frames_to_write):
                video_writer.write(frame)
                frame_count += 1
            last_frame_time = current_time

            frame_processing_time = time.time() - frame_start_time

            if frame_processing_time > sample_interval:
                rospy.logwarn("[Video] Warning: Frame processing is taking longer than the expected frame interval.")
        else:
            rospy.logerr("[Video] No image received from the camera")
        del response, np_response, frame
