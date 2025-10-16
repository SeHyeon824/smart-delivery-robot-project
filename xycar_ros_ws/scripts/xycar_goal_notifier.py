#!/usr/bin/env python
# xycar_goal_notifier.py - 목적지 도착 시 라즈베리파이에 소켓 신호 전송

import rospy
from actionlib_msgs.msg import GoalStatusArray
import socket

# 라즈베리파이 IP와 포트 (환경에 맞게 수정)
PI_IP = '192.168.142.20'  # ← 여기를 라즈베리파이 IP로
PI_PORT = 8000          # smart_safe_main.py와 동일하게

def send_goal_reached_signal():
    try:
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client_socket.connect((PI_IP, PI_PORT))
        client_socket.send("GOAL_REACHED".encode())  # ← 여기 수정된 부분
        client_socket.close()
        rospy.loginfo("✅ 목적지 도착! 라즈베리파이에 신호 전송 완료")
    except Exception as e:
        rospy.logerr(f"[ERROR] Raspberry Pi 연결 실패: {e}")

def callback(data):
    for status in data.status_list:
        if status.status == 3:  # 3 = Goal reached
            rospy.loginfo("📍 MoveBase: 목적지 도착 확인됨")
            send_goal_reached_signal()

def listener():
    rospy.init_node('goal_notifier', anonymous=True)
    rospy.Subscriber("/move_base/status", GoalStatusArray, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
