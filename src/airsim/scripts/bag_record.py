#!/usr/bin/python3
import json
import os
import datetime
import subprocess
import time

import rospy
import rosbag
from std_msgs.msg import Int8

rospy.init_node('record_bags')

while not rospy.is_shutdown():
    if rospy.get_param('/reset', False):
        break

now = datetime.datetime.now()
name_original = now.strftime("%Y%m%d_%H%M%S")

yaml_config_path = rospy.get_param('/setup/config_path', None)
if yaml_config_path:
    file_name_without_extension = os.path.splitext(os.path.basename(yaml_config_path))[0]
    name_original = f"{name_original}_{file_name_without_extension}"
tmp_name = f"__{name_original}"

base_dir = rospy.get_param('~base_dir', "/tmp/airsim")
record_length = rospy.get_param('~record_length', 120)
max_record = rospy.get_param('~max_record', 1)
virtual_speedup = rospy.get_param('~virtual_speedup', 1)

record_length *= virtual_speedup
rospy.loginfo(f"Record {record_length} seconds")

if not os.path.exists(base_dir):
    os.makedirs(base_dir)
print(f"Saving to {os.path.abspath(base_dir)}")

dir_name = os.path.join(base_dir, tmp_name)
dir_name_original = os.path.join(base_dir, name_original)

if not os.path.exists(dir_name):
    os.makedirs(dir_name)

rospy.set_param('/record/dir_name', dir_name)
rospy.set_param('/record/max_record', max_record)
rospy.set_param('/record/max_record', record_length)
rospy.loginfo("[bag] Init record_bags node")

while not rospy.is_shutdown():
    if rospy.get_param('/record/start_record', False):
        params = rospy.get_param_names()
        d = {k: str(rospy.get_param(k)) for k in params}
        with open(os.path.join(dir_name, 'params.json'), 'w') as f:
            json.dump(d, f)
        break

rospy.loginfo(f"[bag] Record Start...")

last_i = -1
publisher = rospy.Publisher('/record/section', Int8, queue_size=10)
start_time = time.time()
bag_name = os.path.join(dir_name, f'all.bag')
rospy.loginfo(f"[bag] Rosbag full file name {bag_name}...")

drone_topics = " ".join(["/drone_{}_visual_slam/odom".format(i) for i in range(14)])
additional_topics = "/search/survivor_found /search/percentage"

# command = "rosbag record -a -O {} --duration={}s".format(bag_name, max_record * record_length)
command = "rosbag record -O {} --duration={}s {} {}".format(bag_name, max_record * record_length,
                                                            drone_topics,
                                                            additional_topics)
process = subprocess.Popen(command, shell=True)

while not rospy.is_shutdown():
    if time.time() - start_time > max_record * record_length:
        break
    i = int((time.time() - start_time) / record_length)
    if i > last_i:
        rospy.loginfo(f"\n--- Start record part {i}/{max_record} ---\n")
        last_i = i
    rospy.sleep(1)
    publisher.publish(i)

process.wait()
process.terminate()
rospy.loginfo(f"[bag] Record Finished...")
rospy.set_param('/record/start_record', False)
rospy.set_param('/general/success_run', 1)

rospy.sleep(5)
params = rospy.get_param_names()
d = {k: str(rospy.get_param(k)) for k in params}
with open(os.path.join(dir_name, 'params_finished.json'), 'w') as f:
    json.dump(d, f)

os.rename(dir_name, dir_name_original)
rospy.loginfo(f"[bag] Rename {dir_name} to {dir_name_original}...")
if not rospy.is_shutdown():
    rospy.signal_shutdown("Recording finished")
    exit(0)

exit(0)
