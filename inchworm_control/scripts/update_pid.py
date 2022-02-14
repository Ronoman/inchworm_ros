import rospy

from inchworm_hw_interface.msg import PID, PIDConsts

last_consts = None

def pid_cb(msg):
    global last_consts

    print("New PID values received:")
    print(msg)

    last_consts = msg

def main():
    pid_pub = rospy.Publisher("update_pid", PIDConsts, queue_size=1)

    while last_consts is None:
        rospy.logwarn("Waiting for first PID consts...")
        rospy.sleep(1)

    print("Consts loaded. Current constants:")
    print(last_consts)

    while not rospy.is_shutdown():
        motor = int(input("Motor (0-4): "))
        forward = bool(input("Forward? (y/n): ").lower() == "y")

        p = float(input("P: "))
        i = float(input("I: "))
        d = float(input("D: "))

        pid = PID(p, i, d)

        old_msg = last_consts

        if forward:
            old_msg.forward[motor] = pid
        else:
            old_msg.backward[motor] = pid

        print("Current consts:")
        print(last_consts)
        print("New consts:")
        print(old_msg)

        pid_pub.publish(old_msg)


if __name__ == "__main__":
    main()