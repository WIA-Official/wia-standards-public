# MotorCommand.msg
# WIA Smart Wheelchair 모터 직접 제어 명령
#
# 이 메시지는 좌/우 모터를 직접 제어하기 위한 명령을 포함합니다.

std_msgs/Header header

# 제어 모드
uint8 mode
uint8 MODE_COAST=0                    # 프리휠 (전류 차단)
uint8 MODE_VELOCITY=1                 # 속도 제어 (rad/s)
uint8 MODE_POSITION=2                 # 위치 제어 (rad)
uint8 MODE_TORQUE=3                   # 토크 제어 (Nm)
uint8 MODE_BRAKE=4                    # 브레이크 (능동 제동)

# 좌측 모터 명령
float32 left_setpoint                 # 설정값 (모드에 따라 해석)
float32 left_acceleration             # 가속도 제한 (rad/s^2)

# 우측 모터 명령
float32 right_setpoint                # 설정값 (모드에 따라 해석)
float32 right_acceleration            # 가속도 제한 (rad/s^2)

# 플래그
bool enable                           # 모터 활성화 여부
bool sync                             # 양쪽 모터 동기화 여부
