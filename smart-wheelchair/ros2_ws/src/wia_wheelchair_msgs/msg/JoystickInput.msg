# JoystickInput.msg
# WIA Smart Wheelchair 처리된 조이스틱 입력

std_msgs/Header header

# 위치 (정규화: -1.0 ~ 1.0)
float32 x                             # 좌우 방향 (양수=우)
float32 y                             # 전후 방향 (양수=전)

# 버튼 상태 (비트마스크)
uint16 buttons
uint16 BUTTON_1=1                     # 버튼 1 (보통 속도 부스트)
uint16 BUTTON_2=2                     # 버튼 2 (호른/경고음)
uint16 BUTTON_3=4                     # 버튼 3 (모드 변경)
uint16 BUTTON_4=8                     # 버튼 4 (비상 정지)
uint16 BUTTON_MODE_UP=16              # 모드 업
uint16 BUTTON_MODE_DOWN=32            # 모드 다운
uint16 BUTTON_PROFILE=64              # 프로파일 선택

# 입력 모드
uint8 input_mode
uint8 INPUT_MODE_STANDARD=0           # 표준
uint8 INPUT_MODE_FINE=1               # 정밀 (감속)
uint8 INPUT_MODE_TURBO=2              # 터보 (증속)

# 프로파일
uint8 profile_id                      # 사용자 프로파일 ID
