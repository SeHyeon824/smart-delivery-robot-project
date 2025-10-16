#!/usr/bin/env python3
import rospy
from xycar_msgs.msg import xycar_motor
import sys
import tty
import termios
import select

def getKey():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        rlist, _, _ = select.select([fd], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key

def main():
    rospy.init_node('xycar_keyboard_controller')
    pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)

    motor_msg = xycar_motor()
    motor_msg.angle = 0
    motor_msg.speed = 0

    speed_step = 3
    angle_step = 5
    max_speed = 50
    min_speed = -50
    max_angle = 50
    min_angle = -50

    rate = rospy.Rate(20)

    print("=======================")
    print("키보드 제어 시작 (1회 누를 때마다 일정 변화)")
    print("w: 가속(+), x: 감속(-)")
    print("a: 좌회전(+), d: 우회전(-)")
    print("space: 속도만 0, s: 속도+조향 모두 0")
    print("q: 종료")
    print("=======================")

    while not rospy.is_shutdown():
        key = getKey()

        if key == 'w':
            motor_msg.speed += speed_step
        elif key == 'x':
            motor_msg.speed -= speed_step
        elif key == 'a':
            motor_msg.angle -= angle_step
        elif key == 'd':
            motor_msg.angle += angle_step
        elif key == ' ':
            motor_msg.speed = 0
        elif key == 's':
            motor_msg.speed = 0
            motor_msg.angle = 0
        elif key == 'q':
            print("종료합니다.")
            break

        # 속도, 조향값 제한
        motor_msg.speed = max(min(motor_msg.speed, max_speed), min_speed)
        motor_msg.angle = max(min(motor_msg.angle, max_angle), min_angle)

        # 현재 상태 출력
        print(f"Speed: {motor_msg.speed}, Angle: {motor_msg.angle}")

        # 퍼블리시
        pub.publish(motor_msg)

        rate.sleep()

if __name__ == '__main__':
    main()
