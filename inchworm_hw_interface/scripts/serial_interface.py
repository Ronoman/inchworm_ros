#!/usr/bin/env python3

import subprocess
import serial
import math

import rospy

from sensor_msgs.msg import JointState

ser = None

# List of 5 element arrays representing joint states at a timestamp (corresponds with the index in timestamps)
expected_joint_angles = []
actual_joint_angles = []

timestamps = []

def jointStateCB(msg):
    global ser, expected_joint_angles, actual_joint_angles

    joints = [math.degrees(j) for j in msg.position]
    expected_joint_angles.append(joints)

    serial_string = ""

    for i,j in enumerate(joints):
        to_add = ""

        to_add += "{:.2f}".format(j)

        # If its a positive joint value, we need to add a padding space
        if j >= 0:
            to_add = " " + to_add.zfill(6) + " "
        else:
            to_add = to_add.zfill(7) + " "

        print(f"Joint {i} string: {to_add}")

        serial_string += to_add

    serial_string += "0 0"

    print(f"Writing:\n\t{serial_string}")
    ser.write(serial_string.encode())

    read_string = str(ser.readline(), encoding="utf8").strip("\r\n")

    print(f"Read:\n\t{read_string}")

    joint_vals = [float(j) for j in read_string.split(" ")[:-2] if not j == ""]

    if not len(joint_vals) == 5:
        return
    
    # print(joint_vals)

    actual_joint_angles.append(joint_vals)
    timestamps.append(rospy.Time.now())


def plot():
    # 5 element array, where each element is a single joint. Indexes into these sub arrays correspond with timestamps array
    joints = []

    # For each joint (or only for a specific subset of joints):
        # Convert into flat list of positions



    # Convert timestamps into floats that start at 0
    # Plot x,y data
    # Legend
    # Axis labels and title
    # Plot

    pass

if __name__ == "__main__":
    rospy.init_node("serial_interface")

    out = subprocess.run(["ls", "/dev"], capture_output=True)

    devices = str(out.stdout, encoding="utf8").split("\n")

    acm_devices = [d for d in devices if "ACM" in d]
    
    device = ""

    if len(acm_devices) == 1:
        device = acm_devices[0]
    else:
        print("Valid devices:")
        print("\n".join(acm_devices))

        device = acm_devices[int(input("Which device would you like? Please type the index from the list (0-indexed): "))]

    print(device)

    ser = serial.Serial("/dev/" + device)

    joint_sub = rospy.Subscriber("/inchworm/joint_states", JointState, jointStateCB, queue_size=1)

    rospy.spin()

    # while not rospy.is_shutdown():
    #     line = str(ser.readline(), encoding="utf8").strip("\r\n")

    #     print(line)