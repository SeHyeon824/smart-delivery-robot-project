#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import tf
import actionlib
import math
import socket # Added for network communication
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionResult
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry

class DestinationWatcher:
    def __init__(self):
        rospy.init_node('destination_watcher')

        self.tf_listener = tf.TransformListener()
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.status_sub = rospy.Subscriber('/move_base/result', MoveBaseActionResult, self.status_callback)
        self.goal_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)

        self.last_position = None
        self.last_move_time = rospy.Time.now()
        self.stuck_timeout = rospy.Duration(3.0)

        self.last_sent_goal = None
        self.retrying = False
        
        # Flag to ensure the goal signal is sent only once per successful destination.
        self.is_goal_signal_sent = False

        rospy.loginfo("Connecting to move_base...")
        self.move_base_client.wait_for_server()
        rospy.loginfo("Connected to move_base.")
        rospy.loginfo("Destination Watcher with goal subscription and stuck recovery started.")

    def goal_callback(self, msg):
        """ Called when a new Nav Goal is published, e.g., from RViz """
        rospy.loginfo("Received new goal from RViz.")
        
        # Reset the flag whenever a new goal is set.
        self.is_goal_signal_sent = False
        self.retrying = False # Also reset the retry flag
        self.last_move_time = rospy.Time.now() # Reset stuck timer
        
        self.send_goal(msg)

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        current = (pos.x, pos.y)

        if self.last_position is None:
            self.last_position = current
            self.last_move_time = rospy.Time.now()
            return

        dist = math.hypot(current[0] - self.last_position[0], current[1] - self.last_position[1])
        if dist > 0.02:
            self.last_position = current
            self.last_move_time = rospy.Time.now()

        if rospy.Time.now() - self.last_move_time > self.stuck_timeout and not self.retrying:
            rospy.logwarn("Robot seems stuck. Initiating backup...")
            self.retrying = True
            self.perform_backup_and_retry()

    def status_callback(self, msg):
        # Check for SUCCEEDED status (3)
        if msg.status.status == 3 and not self.is_goal_signal_sent:
            rospy.loginfo("Goal reached successfully!")
            self.send_signal_to_pi("GOAL_REACHED")
            self.is_goal_signal_sent = True # Set flag to prevent re-sending

        # Check for ABORTED status (4)
        if msg.status.status == 4:
            rospy.logwarn("Goal aborted. Attempting backup and retry...")
            if not self.retrying:
                self.retrying = True
                self.perform_backup_and_retry()
                
    def send_signal_to_pi(self, signal):
        """ Sends a string signal to the Raspberry Pi over a socket. """
        PI_IP = "192.168.142.250"  # IP address of your Raspberry Pi
        PI_PORT = 8000             # The port number your Pi is listening on
        
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.settimeout(3) # Set a 3-second timeout
                s.connect((PI_IP, PI_PORT))
                s.sendall(signal.encode())
                rospy.loginfo(f"Successfully sent signal '{signal}' to the Raspberry Pi.")
        except Exception as e:
            rospy.logerr(f"Failed to send signal to the Raspberry Pi: {e}")

    def perform_backup_and_retry(self):
        self.backup_robot(distance=0.7, speed=-0.15)
        rospy.sleep(1.0)
        self.resend_goal()
        self.retrying = False
        self.last_move_time = rospy.Time.now()

    def backup_robot(self, distance, speed):
        rospy.loginfo(f"Backing up {distance} meters...")
        duration = abs(distance / speed)
        rate = rospy.Rate(10)
        start_time = rospy.Time.now()

        twist = Twist()
        twist.linear.x = speed

        while rospy.Time.now() - start_time < rospy.Duration(duration):
            self.cmd_vel_pub.publish(twist)
            rate.sleep()

        self.cmd_vel_pub.publish(Twist())
        rospy.loginfo("Backup complete.")

    def send_goal(self, pose_stamped):
        """ Sends a goal to the move_base action server. """
        goal = MoveBaseGoal()
        goal.target_pose = pose_stamped
        self.last_sent_goal = goal
        self.move_base_client.send_goal(goal)
        rospy.loginfo("Goal sent to move_base.")

    def resend_goal(self):
        if self.last_sent_goal:
            rospy.logwarn("Resending last goal...")
            self.last_sent_goal.target_pose.header.stamp = rospy.Time.now()
            self.move_base_client.send_goal(self.last_sent_goal)
        else:
            rospy.logwarn("No saved goal to resend.")

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    try:
        dw = DestinationWatcher()
        dw.run()
    except rospy.ROSInterruptException:
        pass
