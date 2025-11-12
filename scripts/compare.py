#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
import numpy as np
import matplotlib.pyplot as plt
import argparse
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

# --- command line arguments ---
parser = argparse.ArgumentParser(description='Compare Velocity Ctrl and Velocity Ctrl Null for a specific joint')
parser.add_argument('--joint', type=int, default=0, help='Joint index to compare (0-6)')
args = parser.parse_args()
joint_index = args.joint

pkg_share = get_package_share_directory('ros2_kdl_package')
bag1_path = os.path.join(pkg_share, 'bags', 'ctrl_bag')
bag2_path = os.path.join(pkg_share, 'bags', 'ctrlnull_bag')

joint_topic = '/joint_states'
cmd_topic = '/velocity_controller/commands'

def read_bag(bag_path, joint_topic, cmd_topic):
    storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = ConverterOptions('', '')
    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    data = {
        'time_joint': [],
        'position': [],
        'velocity': [],
        'time_cmd': [],
        'cmd_vel': []
    }

    while reader.has_next():
        (topic, data_bytes, t) = reader.read_next()
        t_sec = t / 1e9  # ns -> s

        if topic == joint_topic:
            msg = deserialize_message(data_bytes, JointState)
            data['time_joint'].append(t_sec)
            data['position'].append(msg.position)
            data['velocity'].append(msg.velocity)

        elif topic == cmd_topic:
            msg = deserialize_message(data_bytes, Float64MultiArray)
            data['time_cmd'].append(t_sec)
            data['cmd_vel'].append(msg.data)

    # conversion to numpy arrays
    data['position'] = np.array(data['position'])
    data['velocity'] = np.array(data['velocity'])
    data['cmd_vel'] = np.array(data['cmd_vel'])
    data['time_joint'] = np.array(data['time_joint'])
    data['time_cmd'] = np.array(data['time_cmd'])

    # normalize time starting at 0
    if len(data['time_joint']) > 0:
        t0_joint = data['time_joint'][0]
        data['time_joint'] -= t0_joint
    if len(data['time_cmd']) > 0:
        t0_cmd = data['time_cmd'][0]
        data['time_cmd'] -= t0_cmd

    return data

# read data from both bags
data1 = read_bag(bag1_path, joint_topic, cmd_topic)
data2 = read_bag(bag2_path, joint_topic, cmd_topic)

# --- plot position ---
plt.figure(figsize=(12,5))
plt.plot(data1['time_joint'], data1['position'][:, joint_index], label=f'Joint {joint_index} Position - Vel Ctrl')
plt.plot(data2['time_joint'], data2['position'][:, joint_index], '--', label=f'Joint {joint_index} Position - Vel Ctrl Null')
plt.xlabel('Time [s]')
plt.ylabel('Position [rad]')
plt.title(f'Joint {joint_index} Position Comparison')
plt.legend()
plt.grid(True)
plt.show()

# --- plot commanded velocity ---
plt.figure(figsize=(12,5))
plt.plot(data1['time_cmd'], data1['cmd_vel'][:, joint_index], label=f'Joint {joint_index} Commanded Velocity - Vel Ctrl')
plt.plot(data2['time_cmd'], data2['cmd_vel'][:, joint_index], '--', label=f'Joint {joint_index} Commanded Velocity - Vel Ctrl Null')
plt.xlabel('Time [s]')
plt.ylabel('Commanded Velocity [rad/s]')
plt.title(f'Joint {joint_index} Commanded Velocity Comparison')
plt.legend()
plt.grid(True)
plt.show()
