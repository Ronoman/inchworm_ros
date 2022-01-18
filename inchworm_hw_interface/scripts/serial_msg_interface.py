#!/usr/bin/env python3

import serial, rospy, subprocess, sys, struct, inspect

from threading import Lock

from std_msgs.msg import Int32, String
from sensor_msgs.msg import JointState

from inchworm_hw_interface.msg import MagnetState, PIDConsts, PID

# Baud rate to communicate with the Teensy
BAUD = 9600

# If True, then the serial port is not connected. All writes are simply logged
DEBUG = False

# Publishers for each kind of message embedded software can send to us
heartbeat_pub = None
joint_poses_pub = None
pid_consts_pub = None
magnet_state_pub = None
debug_pub = None
fault_pub = None

# Subscribers for each kind of message we can send to embedded software
heartbeat_sub = None
joint_goal_sub = None
pid_consts_sub = None
magnet_state_sub = None

serial_port = None
ser_mutex = Lock()

def send_serial(to_send):
    if DEBUG:
        suppress = ["send_joint_goal"]

        if not inspect.stack()[1].function in suppress:
            print(f"Function {inspect.stack()[1].function} sending:")
            print(to_send)
            print()
    else:
        ser_mutex.acquire()
        serial_port.write(to_send)
        ser_mutex.release()

def heartbeat(byte_arr):
    seq = struct.unpack(">i", byte_arr)

    heartbeat_msg = Int32(seq)

    heartbeat_pub.publish(heartbeat_msg)

def joint_poses(byte_arr):
    # Unpack 10 contiguous doubles
    poses = struct.unpack(">" + "d"*10, byte_arr)

    joint_state_msg = JointState()

    joint_state_msg.header.stamp = rospy.Time.now()

    # TODO: Figure out the naming convention for joints, because this is bad and inconsistent
    joint_state_msg.name = ["j0", "j1", "j2", "j3", "j4"]

    for pos in poses[:5]:
        joint_state_msg.position.append(pos)

    for effort in poses[5:]:
        joint_state_msg.effort.append(effort)

    joint_poses_pub.publish(joint_state_msg)

def pid_consts(byte_arr):
    consts = struct.unpack(">" + "d"*30, byte_arr)

    tuples = ((consts[3*i], consts[3*i+1], consts[3*i+2]) for i in range(len(consts)))

    consts_msg = PIDConsts()

    for i in range(5):
        forward = PID()
        backward = PID()

        forward.p = tuples[i][0]
        forward.i = tuples[i][1]
        forward.d = tuples[i][2]

        backward.p = tuples[i+5][0]
        backward.i = tuples[i+5][1]
        backward.d = tuples[i+5][2]

        consts_msg.forward.append(forward)
        consts_msg.backward.append(backward)

    pid_consts_pub.publish(consts_msg)

def magnet_state(byte_arr):
    mag_states = struct.unpack(">ii", byte_arr)

    mag_state_msg = MagnetState()

    mag_state_msg.magnet1 = mag_states[0]
    mag_state_msg.magnet2 = mag_states[1]

    magnet_state_pub.publish(mag_state_msg)

def debug(byte_arr):
    message = struct.unpack(">100s")

    debug_msg = String(message)

    debug_pub.publish(debug_msg)

def fault(byte_arr):
    message = struct.unpack(">100s")

    fault_msg = String(message)

    fault_pub.publish(fault_msg)

def send_heartbeat(msg):
    to_send = struct.pack(">cxxxi", b"h", msg.data)

    send_serial(to_send)

def send_joint_goal(msg):
    to_send = struct.pack(">cxxx" + "d"*10, b"g", *msg.position, *msg.effort)

    send_serial(to_send)

def send_pid_consts(msg):
    consts_arr = []

    f = msg.forward
    b = msg.backward

    if not len(f) == len(b):
        rospy.logerr(f"send_pid_consts: Mismatch on joint count. f: {f} b: {b}")

    # len(f) === len(b) by now, so we can just check one
    if len(f) != 5:
        rospy.logerr(f"send_pid_consts: Got {len(f)} joints, needed 5")
        return

    for pid in f:
        consts_arr.extend([pid.p, pid.i, pid.d])
    for pid in b:
        consts_arr.extend([pid.p, pid.i, pid.d])

    to_send = struct.pack(">cxxx" + "d"*30, b"p", *consts_arr)

    send_serial(to_send)

def send_magnet_states(msg):
    to_send = struct.pack(">cxxxii", b"m", msg.magnet1, msg.magnet2)

    send_serial(to_send)

def init_pubs():
    global heartbeat_pub, joint_poses_pub, pid_consts_pub, magnet_state_pub, debug_pub, fault_pub

    heartbeat_pub = rospy.Publisher("heartbeat_res", Int32, queue_size=1)
    joint_poses_pub = rospy.Publisher("joint_states", JointState, queue_size=1)
    pid_consts_pub = rospy.Publisher("pid_consts", PIDConsts, queue_size=1)
    magnet_state_pub = rospy.Publisher("magnet_states", MagnetState, queue_size=1)
    debug_pub = rospy.Publisher("debug", String, queue_size=1)
    fault_pub = rospy.Publisher("fault", String, queue_size=1)

def init_subs():
    global heartbeat_sub, joint_goal_sub, pid_consts_sub, magnet_state_sub

    heartbeat_sub = rospy.Subscriber("heartbeat_req", Int32, send_heartbeat, queue_size=5)
    joint_goal_sub = rospy.Subscriber("inchworm/joint_states", JointState, send_joint_goal, queue_size=5)
    pid_consts_sub = rospy.Subscriber("update_pid", PIDConsts, send_pid_consts, queue_size=5)
    magnet_state_sub = rospy.Subscriber("set_magnet_states", MagnetState, send_magnet_states, queue_size=5)

def init_serial():
    global DEBUG

    out = subprocess.run(["ls", "/dev"], capture_output=True)

    devices = str(out.stdout, encoding="utf8").split("\n")

    acm_devices = [d for d in devices if "ACM" in d]
    
    device = ""

    # If there are no devices matching the pattern, assume we should be in debug mode
    if len(acm_devices) == 0:
        DEBUG = True
        return None

    elif len(acm_devices) == 1:
        device = acm_devices[0]

    else:
        print("Valid devices:")
        print("\n".join(acm_devices))

        device = acm_devices[int(input("Which device would you like? Please type the index from the list (0-indexed): "))]

    print(device)

    ser = serial.Serial("/dev/" + device, BAUD, timeout=2)

    return ser

# Maps type characters from incoming serial messages to parser functions
# Key: Type character
# Value: (message_size, handler_fn)
# More info in https://docs.google.com/document/d/1m7oZZbM0VJFrIxo1KRNSXnIhmvcEmCzY7amgNRJE87Q/edit?usp=sharing
char_fn_map = {
    "h": (8, heartbeat),
    "j": (84, joint_poses),
    "p": (244, pid_consts),
    "m": (12, magnet_state),
    "d": (104, debug),
    "f": (104, fault)
}

if __name__ == "__main__":
    rospy.init_node("message_parser")

    init_pubs()
    init_subs()

    serial_port = init_serial()

    while not rospy.is_shutdown():
        # Read in the type char if the serial port initialized
        if serial_port is not None:
            ser_mutex.acquire()
            type_char = str(serial_port.read(1), encoding="utf8").string("\r\n")
            ser_mutex.release()
        else:
            rospy.sleep(1)
            continue

        if type_char in char_fn_map:
            # Skip 3 padding bytes
            ser_mutex.acquire()
            _ = serial_port.read(3)
            ser_mutex.release()

            # Read in the specified number of bytes, minus 4 for type_char+3*padding bytes
            byte_arr = serial_port.read(char_fn_map[type_char][0] - 4)

            # Pass the byte array to the appropriate handler
            char_fn_map[type_char][1](byte_arr)
        else:
            rospy.logerr(f"Invalid message received from robot with type_char {type_char}. Quitting")
            sys.exit()