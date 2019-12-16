#!/usr/bin/python
import intera_interface
import rospy
import std_msgs

g_gripper = None
g_sub_open = None
g_sub_close = None
g_is_pneumatic = False

def init():
    global g_sub_open, g_sub_close, g_gripper, g_is_pneumatic

    g_sub_open = rospy.Subscriber('/cairo/sawyer_gripper_open', std_msgs.msg.Empty, open_grip)
    g_sub_close = rospy.Subscriber('/cairo/sawyer_gripper_close', std_msgs.msg.Empty, close_grip)

    g_gripper = intera_interface.get_current_gripper_interface()
    g_is_pneumatic = isinstance(g_gripper, intera_interface.SimpleClickSmartGripper)

    if g_is_pneumatic:
        if g_gripper.needs_init():
            g_gripper.initialize()
            if g_gripper.needs_init(): return False
    else:
        if not g_gripper.is_calibrated():
            g_gripper.calibrate()
            if not g_gripper.is_calibrated(): return False

    return True

def open_grip(msg):
    global g_gripper, g_is_pneumatic
    rospy.loginfo("Opening gripper")
    if g_is_pneumatic:
        g_gripper.set_ee_signal_value('grip', False)
    else:
        g_gripper.open()

def close_grip(msg):
    global g_gripper, g_is_pneumatic

    rospy.loginfo("Closing gripper")
    if g_is_pneumatic:
        g_gripper.set_ee_signal_value('grip', True)
    else:
        g_gripper.close()


def main():
    rospy.init_node("sawyer_gripper_wrapper")
    if init():
        rospy.loginfo("Sawyer Gripper Wrapper Initialized")
        rospy.spin()
    else:
        rospy.logerr("Couldn't initialize Sawyer Gripper")


if __name__ == '__main__':
    main()
