# SafetyStatus.msg
# WIA Smart Wheelchair 안전 시스템 상태

std_msgs/Header header

# 비상 정지
bool emergency_stop_active            # 비상 정지 활성화 여부
uint8 emergency_stop_source           # 비상 정지 원인
uint8 ESTOP_SOURCE_NONE=0
uint8 ESTOP_SOURCE_BUTTON=1           # 물리 버튼
uint8 ESTOP_SOURCE_REMOTE=2           # 원격 명령
uint8 ESTOP_SOURCE_SOFTWARE=3         # 소프트웨어 감지
uint8 ESTOP_SOURCE_SENSOR=4           # 센서 이상

# 장애물 감지
bool obstacle_front                   # 전방 장애물
bool obstacle_left                    # 좌측 장애물
bool obstacle_right                   # 우측 장애물
bool obstacle_rear                    # 후방 장애물
float32 min_obstacle_distance         # 최소 장애물 거리 (m)

# 기울기 상태
float32 tilt_angle                    # 현재 기울기 (degrees)
bool tilt_warning                     # 기울기 경고
bool tilt_critical                    # 기울기 위험

# 시스템 상태
bool system_healthy                   # 전체 시스템 정상
uint8 safety_level                    # 안전 수준
uint8 SAFETY_LEVEL_NORMAL=0           # 정상
uint8 SAFETY_LEVEL_CAUTION=1          # 주의
uint8 SAFETY_LEVEL_WARNING=2          # 경고
uint8 SAFETY_LEVEL_CRITICAL=3         # 위험
