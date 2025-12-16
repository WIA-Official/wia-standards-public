# MotorStatus.msg
# WIA Smart Wheelchair 개별 모터 상태

std_msgs/Header header

# 식별
string motor_id                       # "left" 또는 "right"

# 현재 상태
uint8 mode                            # 현재 제어 모드
float32 velocity                      # 현재 속도 (rad/s)
float32 position                      # 현재 위치 (rad)
float32 torque                        # 현재 토크 (Nm)
float32 current                       # 전류 (A)

# 환경
int8 temperature                      # 온도 (°C)

# 상태
bool enabled                          # 활성화 여부
bool fault                            # 폴트 상태
uint8 error_code                      # 에러 코드
