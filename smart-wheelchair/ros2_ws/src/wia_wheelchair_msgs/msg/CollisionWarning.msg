# CollisionWarning.msg
# WIA Smart Wheelchair 충돌 경고

std_msgs/Header header

# 충돌 위험
bool collision_imminent               # 즉각적인 충돌 위험
float32 time_to_collision             # 충돌까지 예상 시간 (seconds)
float32 distance_to_obstacle          # 장애물까지 거리 (m)

# 위치 정보
geometry_msgs/Point obstacle_position # 장애물 위치 (base_link 기준)
float32 obstacle_angle                # 장애물 방향 (rad, 전방=0)

# 권장 동작
uint8 recommended_action
uint8 ACTION_NONE=0                   # 조치 불필요
uint8 ACTION_SLOW_DOWN=1              # 감속 권장
uint8 ACTION_STOP=2                   # 정지 권장
uint8 ACTION_REVERSE=3                # 후진 권장

# 경고 레벨
uint8 warning_level
uint8 LEVEL_INFO=0                    # 정보
uint8 LEVEL_WARNING=1                 # 경고
uint8 LEVEL_DANGER=2                  # 위험
uint8 LEVEL_CRITICAL=3                # 긴급
