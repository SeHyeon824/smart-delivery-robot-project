#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist       # /cmd_vel 토픽의 메시지 타입
from xycar_msgs.msg import xycar_motor  # 자이카 모터 제어를 위한 메시지 타입

# === 제어 값 설정 (중요: 이 값들은 자이카의 특성 및 실험을 통해 꼭! 조절해야 합니다!) ===

# /cmd_vel의 linear.x (m/s) 값을 xycar_motor의 speed 값으로 변환하기 위한 배율
# 예: linear.x가 0.5 m/s일 때 speed를 50으로 하고 싶다면 100.0
# 이 값은 로봇의 최대 속도와 /cmd_vel의 최대 linear.x 값을 고려하여 정합니다.
SPEED_SCALE_FACTOR = 100.0  # <--- 이 값을 조절하세요!

# /cmd_vel의 angular.z (rad/s) 값을 xycar_motor의 angle 값(아마도 도, degree)으로 변환하기 위한 배율
# angular.z의 양수 값은 ROS 표준에서 보통 반시계 방향 (좌회전)입니다.
# xycar_motor의 angle 값이 어떤 방향을 기준으로 하는지, 단위가 무엇인지(degree인지 radian인지 등) 확인이 필요합니다.
# 만약 xycar_motor의 angle이 양수일 때 우회전이라면, 아래 계산식에서 부호(-)를 유지하거나 조정해야 합니다.
# 또한, angular.z는 각속도(rad/s)이고, angle은 특정 조향각일 가능성이 높으므로,
# 이 배율은 로봇의 조향 특성(얼마나 꺾이는지)에 맞게 신중히 조절해야 합니다.
ANGLE_SCALE_FACTOR = 30.0   # <--- 이 값을 조절하세요! (단위 및 방향 고려)

# 자이카 모터 메시지의 최대/최소 값 (이전 키보드 제어 스크립트의 값을 참고)
MAX_SPEED = 50
MIN_SPEED = -50 # 후진을 허용하지 않으려면 0으로 설정할 수도 있습니다.
MAX_ANGLE = 50
MIN_ANGLE = -50
# ==========================================================================

motor_pub = None # motor_pub을 전역 변수로 선언

def cmd_vel_callback(twist_msg):
    global motor_pub # 전역 변수 motor_pub 사용 명시

    # motor_pub이 초기화되지 않았으면 함수 종료 (오류 방지)
    if motor_pub is None:
        rospy.logwarn("Motor publisher is not initialized yet.")
        return

    # xycar_motor 메시지 객체 생성
    motor_msg = xycar_motor()

    # === 속도 변환 ===
    # twist_msg.linear.x는 전진(+) 또는 후진(-) 속도를 나타냅니다 (단위: m/s).
    # SPEED_SCALE_FACTOR를 곱해서 자이카의 speed 값으로 변환합니다.
    calculated_speed = twist_msg.linear.x * SPEED_SCALE_FACTOR

    # 계산된 속도 값을 MAX_SPEED와 MIN_SPEED 사이로 제한하고 정수형으로 변환
    motor_msg.speed = int(max(min(calculated_speed, MAX_SPEED), MIN_SPEED))

    # === 조향각 변환 ===
    # twist_msg.angular.z는 로봇의 Z축 기준 회전 속도 (단위: rad/s).
    # 양수: 반시계방향 (보통 좌회전), 음수: 시계방향 (보통 우회전)
    # ANGLE_SCALE_FACTOR를 곱해서 자이카의 angle 값으로 변환합니다.
    # 여기서 부호(-)를 곱한 이유: angular.z 양수(좌회전)를 angle 음수(좌회전)로 매칭시키려는 의도 (자이카의 angle 특성에 따라 달라짐!)
    # 만약 자이카의 angle 양수가 좌회전이라면, 부호(-)를 제거해야 합니다.
    calculated_angle = -(twist_msg.angular.z * ANGLE_SCALE_FACTOR) # <--- 자이카의 조향 방향에 맞게 이 부호를 꼭! 확인하고 수정하세요.

    # 계산된 조향각 값을 MAX_ANGLE과 MIN_ANGLE 사이로 제한하고 정수형으로 변환
    motor_msg.angle = int(max(min(calculated_angle, MAX_ANGLE), MIN_ANGLE))
    
    # 디버깅을 위해 현재 수신된 /cmd_vel 값과 변환된 모터 명령 값을 로그로 출력
    rospy.loginfo(f"Received /cmd_vel: linear.x={twist_msg.linear.x:.2f}, angular.z={twist_msg.angular.z:.2f} "
                  f"--> Publishing xycar_motor: speed={motor_msg.speed}, angle={motor_msg.angle}")

    # 최종적으로 변환된 모터 명령을 'xycar_motor' 토픽으로 발행
    motor_pub.publish(motor_msg)

def main():
    global motor_pub # 전역 변수 motor_pub 사용 명시

    # 'cmdvel_to_xycarmotor_node' 라는 이름으로 ROS 노드를 초기화합니다.
    # anonymous=True는 같은 이름의 노드가 여러 개 실행될 수 있도록 해줍니다 (필요에 따라).
    rospy.init_node('cmdvel_to_xycarmotor_node', anonymous=True)

    # '/cmd_vel' 토픽을 구독(Subscribe)하도록 설정합니다.
    # 이 토픽으로 geometry_msgs/Twist 타입의 메시지가 들어오면, cmd_vel_callback 함수가 호출됩니다.
    # queue_size는 메시지 큐의 크기입니다.
    rospy.Subscriber('/cmd_vel', Twist, cmd_vel_callback, queue_size=10)

    # 'xycar_motor' 토픽으로 xycar_msgs/xycar_motor 타입의 메시지를 발행(Publish)할 준비를 합니다.
    # motor_pub 변수에 이 퍼블리셔를 할당합니다.
    motor_pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=10)

    rospy.loginfo("Cmd_vel to Xycar_motor converter node started.")
    rospy.loginfo("Subscribing to /cmd_vel topic.")
    rospy.loginfo("Publishing to /xycar_motor topic.")

    # rospy.spin()은 스크립트가 종료되지 않고 계속 실행되면서,
    # 구독하는 토픽에 메시지가 들어올 때마다 해당 콜백 함수를 호출할 수 있도록 대기합니다.
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        # Ctrl+C 등으로 노드가 종료될 때 발생하는 예외를 처리합니다.
        rospy.loginfo("Cmd_vel to Xycar_motor converter node shut down.")
