# WheelchairState.msg
# WIA Smart Wheelchair 통합 상태 메시지
#
# 이 메시지는 휠체어의 전체 상태를 포함합니다.
# 발행 주기: 50 Hz

std_msgs/Header header

# 운동 상태
geometry_msgs/Twist velocity          # 현재 속도 (linear.x, angular.z)
geometry_msgs/Pose pose               # 현재 위치/자세

# 모터 상태
float32 left_motor_speed              # 좌측 모터 속도 (rad/s)
float32 right_motor_speed             # 우측 모터 속도 (rad/s)
float32 left_motor_current            # 좌측 모터 전류 (A)
float32 right_motor_current           # 우측 모터 전류 (A)
int8 left_motor_temperature           # 좌측 모터 온도 (°C)
int8 right_motor_temperature          # 우측 모터 온도 (°C)

# 배터리 상태
float32 battery_voltage               # 배터리 전압 (V)
float32 battery_current               # 배터리 전류 (A)
float32 battery_soc                   # 충전 상태 (0.0-1.0)
int8 battery_temperature              # 배터리 온도 (°C)

# 운행 모드
uint8 mode                            # 운행 모드
uint8 MODE_MANUAL=0                   # 수동 조작
uint8 MODE_ASSISTED=1                 # 보조 주행
uint8 MODE_AUTONOMOUS=2               # 자율 주행

# 상태 플래그 (비트마스크)
uint8 status
uint8 STATUS_MOTORS_ENABLED=1         # Bit 0: 모터 활성화
uint8 STATUS_JOYSTICK_CONNECTED=2     # Bit 1: 조이스틱 연결
uint8 STATUS_EMERGENCY_STOP=4         # Bit 2: 비상 정지
uint8 STATUS_LOW_BATTERY=8            # Bit 3: 배터리 부족
uint8 STATUS_OBSTACLE_DETECTED=16     # Bit 4: 장애물 감지
uint8 STATUS_TILT_WARNING=32          # Bit 5: 기울기 경고

# 에러 코드
uint16 error_code                     # 에러 코드 (0 = 에러 없음)
