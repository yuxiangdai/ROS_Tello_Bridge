#!/usr/bin/env python
# encoding: UTF-8
# date: March 4, 2020
# author: Yuxiang Dai
# description: Tello ROS interface using Tello-Python official DJI SDK

# import math
# import numpy as np
import rospy
from std_msgs.msg import Int8, String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image

from tellolib.Tello_Video.tello import Tello
from tellolib.Tello_Video.tello_control_ui import TelloUI

import time
import threading
import struct
import cv2
from cv_bridge import CvBridge, CvBridgeError

# copied from https://github.com/hanyazou/TelloPy
from protocol import *

LOOP_FREQ = 20
LOOP_DT = 0.05


class Mode(object):
    LANDED = 0
    FLYING = 1
    UNKNOWN = 2

class TelloROSDriver(object):
    ''' sub scribe topic and publish after safety check
    '''

    def __init__(self, tello):

        self._tello = tello
        self._cmd_vel = Twist()
        self._mode = Mode.LANDED
        self.bridge = CvBridge()
        
        self._img_pub = rospy.Publisher('image_raw', Image, queue_size=10)
        self._cmd_vel_sub = rospy.Subscriber('cmd_vel', Twist, self._cmd_vel_sub_cb)
       
        self._mode_sub = rospy.Subscriber('mode', Int8, self._mode_sub_cb)
         
        # Control Variables
        self.distance = 0.1  # default distance for 'move' cmd
        self.degree = 90  # default degree for 'cw' or 'ccw' cmd

        # Create subscribers 
        self._keys_sub = rospy.Subscriber('keys', String, self._keys_cb)
       
        # Subscriber for commands
        self._command_sub = rospy.Subscriber('command', String, self._command_sub_cb)

        # start a thread that constantly pools the video sensor for
        # the most recently read frame
        self.stopEvent = threading.Event()
        self.thread = threading.Thread(target=self.videoLoop, args=())
        self.thread.start()
    
    def videoLoop(self):
        """
        COPY and modified from tello_control_ui.py in Tello-Python
        The mainloop thread of Tkinter 
        Raises:
            RuntimeError: To get around a RunTime error that Tkinter throws due to threading.
        """
        try:
            # start the thread that get GUI image and drwa skeleton 
            time.sleep(0.5)
            while not self.stopEvent.is_set():  
                start = time.time()              
                # read the frame for GUI show
                self.frame = self._tello.read()
                if self.frame is None or self.frame.size == 0:
                    continue           

                # smoothing filter
                self.frame = cv2.bilateralFilter(self.frame, 5, 50, 100)  
                self._img_pub.publish(self.bridge.cv2_to_imgmsg(self.frame, "rgb8"))
                
                time.sleep(0.04) #todo cant find reason why quality less than tiker
                # print time.time() - start
                # wait = 0.05 - (time.time() - start)
                # if wait > 0:
                #     time.sleep(wait)

        except RuntimeError, e:
            rospy.loginfo('caught a RuntimeError for gettng video')

    def _cmd_vel_sub_cb(self, msg):
        self._cmd_vel = msg

    def _keys_cb(self, msg):
        print(msg.data)

    def _command_sub_cb(self, msg):
        action = msg.data
        print "%s %0.2f m, %d degrees" % (action, self.distance, self.degree)
        if action == "takeoff":
            self.telloTakeOff()
        elif action == "land":
            self.telloLanding()
        elif action == "forward":
            self.telloMoveForward(self.distance)
        elif action == "backward":
            self.telloMoveBackward(self.distance)
        elif action == "left":
            self.telloMoveLeft(self.distance)
        elif action == "right":
            self.telloMoveRight(self.distance)
        elif action == "ccw":
            self.telloCCW(self.degree)
        elif action == "cw":
            self.telloCW(self.degree)
        else:
            print "No matching action"

    def _mode_sub_cb(self, msg):

        self._mode = msg.data
        self._cmd_vel = Twist()

        if self._mode == Mode.LANDED:
            self._tello.land()
        elif self._mode == Mode.FLYING:
            self._tello.takeoff()

    ### Keyboard logic
    def on_keypress_right(self, event):
        # distance * 100 cm
        print "right %d m" % self.distance
        self.telloMoveRight(self.distance)

    def on_keypress_w(self, event):
        print "up %d m" % self.distance
        self.telloUp(self.distance)

    def on_keypress_s(self, event):
        print "down %d m" % self.distance
        self.telloDown(self.distance)

    def on_keypress_a(self, event):
        print "ccw %d degree" % self.degree
        self._tello.rotate_ccw(self.degree)

    def on_keypress_d(self, event):
        print "cw %d m" % self.degree
        self._tello.rotate_cw(self.degree)

    def on_keypress_up(self, event):
        print "forward %d m" % self.distance
        self.telloMoveForward(self.distance)

    def on_keypress_down(self, event):
        print "backward %d m" % self.distance
        self.telloMoveBackward(self.distance)

    def on_keypress_left(self, event):
        print "left %d m" % self.distance
        self.telloMoveLeft(self.distance)

    def on_keypress_right(self, event):
        print "right %d m" % self.distance
        self.telloMoveRight(self.distance)

    ### Tello Movement logic
    def telloTakeOff(self):
        """
        send the takeoff command to tello,and wait for the first response,
        
        if get the 'error'response,remind the "battery low" warning.Otherwise,
        
        start the auto-takeoff thread
        """
        takeoff_response = None

        self._tello.takeoff()
        time.sleep(0.2)

        takeoff_response = self._tello.get_response()

        if takeoff_response != 'error':
            self.auto_takeoff_thread.start()       
        else:
            print "battery low,please repalce with a new one"                          


    def telloLanding(self):
        return self._tello.land()

    # def telloFlip_l(self):
    #     return self._tello.flip('l')

    # def telloFlip_r(self):
    #     return self._tello.flip('r')

    # def telloFlip_f(self):
    #     return self._tello.flip('f')

    # def telloFlip_b(self):
    #     return self._tello.flip('b')

    def telloCW(self, degree):
        return self._tello.rotate_cw(degree)

    def telloCCW(self, degree):
        return self._tello.rotate_ccw(degree)

    def telloMoveForward(self, distance):
        return self._tello.move_forward(distance)

    def telloMoveBackward(self, distance):
        return self._tello.move_backward(distance)

    def telloMoveLeft(self, distance):
        return self._tello.move_left(distance)

    def telloMoveRight(self, distance):
        # distance * 100 cm
        return self._tello.move_right(distance)

    def telloUp(self, dist):
        return self._tello.move_up(dist)

    def telloDown(self, dist):
        return self._tello.move_down(dist)

    def send_packet(self, pkt):
        """Send_packet is used to send a command packet to the drone."""
        try:
            cmd = pkt.get_buffer()
            self._tello.socket.sendto(cmd, self._tello.tello_address)
            rospy.logdebug("send_packet: %s" % byte_to_hexstring(cmd))
        except socket.error as err:
            if self.state == self.STATE_CONNECTED:
                rospy.logerr("send_packet: %s" % str(err))
            else:
                rospy.logerr("send_packet: %s" % str(err))
            return False

        return True

    def __send_stick_command(self, msg):
        '''
        copy and modified from Tellopy https://github.com/hanyazou/TelloPy
        '''
        fast_mode = False

        pkt = Packet(STICK_CMD, 0x60)

        axis1 = int(1024 + 660.0 * -msg.linear.y) & 0x7ff
        axis2 = int(1024 + 660.0 *  msg.linear.x) & 0x7ff
        axis3 = int(1024 + 660.0 *  msg.linear.z) & 0x7ff
        axis4 = int(1024 + 660.0 * -msg.angular.z) & 0x7ff
        axis5 = int(fast_mode) & 0x01
        rospy.logdebug("stick command: fast=%d yaw=%4d vrt=%4d pit=%4d rol=%4d" %
                       (axis5, axis4, axis3, axis2, axis1))

        '''
        11 bits (-1024 ~ +1023) x 4 axis = 44 bits
        fast_mode takes 1 bit
        44+1 bits will be packed in to 6 bytes (48 bits)

         axis5      axis4      axis3      axis2      axis1
             |          |          |          |          |
                 4         3         2         1         0
        98765432109876543210987654321098765432109876543210
         |       |       |       |       |       |       |
             byte5   byte4   byte3   byte2   byte1   byte0
        '''
        packed = axis1 | (axis2 << 11) | (
            axis3 << 22) | (axis4 << 33) | (axis5 << 44)
        packed_bytes = struct.pack('<Q', packed)
        pkt.add_byte(byte(packed_bytes[0]))
        pkt.add_byte(byte(packed_bytes[1]))
        pkt.add_byte(byte(packed_bytes[2]))
        pkt.add_byte(byte(packed_bytes[3]))
        pkt.add_byte(byte(packed_bytes[4]))
        pkt.add_byte(byte(packed_bytes[5]))
        pkt.add_time()
        pkt.fixup()
        rospy.logdebug("stick command: %s" %
                       byte_to_hexstring(pkt.get_buffer()))
        return self.send_packet(pkt)

    def spin(self):
        count = 0
        r = rospy.Rate(LOOP_FREQ)
        while not rospy.is_shutdown():
            if count > 100:
                count = 0
                self._tello.send_command('command')  
            if self._mode == Mode.FLYING:
                cmd = self._cmd_vel    
                self.__send_stick_command(cmd)
            else:
                pass

            count += 1
            r.sleep()
        
        self.stopEvent.set()
        self.thread.join()
        del self._tello

def main():
    tello = Tello('', 8889) 

    rospy.init_node('tello_control')

    # Main loop of tello_control_ui
    # vplayer = TelloUI(tello,"./img/")
    # vplayer.root.mainloop() 
    
    node = TelloROSDriver(tello)
    node.spin()

if __name__ == '__main__':
    main()