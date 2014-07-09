#!/usr/bin/env python

import serial
import string
from time import sleep
from threading import Timer, Thread
from signal import signal, SIGINT
import sys

# ROS stuff
import roslib; roslib.load_manifest("qex")
import rospy
from tf.msg import tfMessage

# Loop time
serial_port = sys.argv[1]
baudrate = 57600
loop_period = 0.001
tx_interval = 50

# Debug
x = 0.0
y = 0.0
z = 0.0

def tf_proc_callback(tf):
    global x, y, z
    frame = tf.transforms[0]
    if frame.child_frame_id == "/map":
        x = frame.transform.rotation.x
        y = frame.transform.rotation.y
        z = frame.transform.rotation.z

# Visualize with dots
def dotvis(n):
    l = 50
    s = 25 + int(25*n)
    e = 25
    if s > 25:
        e = s
        s = 25

    v = "["
    for i in range(s):
        v += ' '
    for i in range(s, e):
        v += '.'
    for i in range(e, l):
        v += ' '
    v += ']'

    return v


def transmit():
    global x, y, z
    print "x:", dotvis(x)
    print "y:", dotvis(y)
    print "z:", dotvis(z)
    print "--"
    # TODO(yoos): Mavlink!


###############################################################################

if __name__ == "__main__":
    # Initialize ROS node.
    rospy.init_node("qex_gs", anonymous=False)
    #pub = rospy.Publisher("topic_name", TODO_MAVLINK)
    sub = rospy.Subscriber("/tf", tfMessage, tf_proc_callback, queue_size=100)

    # TODO(yoos): reenable when we're ready to test with quad
    #try:
    #    ser = serial.Serial(serial_port, baudrate, timeout=0)
    #except:
    #    rospy.logerr("[Comm] Unable to open specified serial port! Exiting...")
    #    exit(1)

    loop_count = 0
    while not rospy.is_shutdown():
        if loop_count % tx_interval == 0:
            transmit()

        loop_count = (loop_count+1) % 1000

        rospy.sleep(loop_period)

