# 🔢 WIA Talk - 제스처 코드 체계

**홍익인간 (弘益人間) - Benefit All Humanity**

---

## 📊 개요 (Overview)

WIA Talk의 93개 제스처를 체계적으로 코드화하는 시스템입니다.

**WIA Braille**이 IPA 음소를 점자 코드로 매핑하듯이,
**WIA Talk**는 제스처 구성 요소를 코드로 매핑합니다.

---

## 🎯 코드 체계 설계 철학

### 1. 계층적 구조 (Hierarchical)
- 카테고리 → 세부 제스처
- 구성 요소 기반 분해
- 확장 가능한 구조

### 2. 인간 가독성 (Human Readable)
- 의미 있는 약어 사용
- 직관적 이해 가능
- 기억하기 쉬운 패턴

### 3. 기계 처리 가능 (Machine Processable)
- 일관된 형식
- 파싱 용이
- 데이터베이스 친화적

### 4. 확장성 (Scalable)
- 새 제스처 추가 용이
- 호환성 유지
- 버전 관리 가능

---

## 📝 코드 체계 A: 카테고리 기반 (Category-Based)

### 형식
```
[카테고리코드]-[번호]
```

### 카테고리 코드

| 카테고리 | 코드 | 개수 | 범위 |
|----------|------|------|------|
| Basic Hand Shapes | **BH** | 10 | BH-01 ~ BH-10 |
| Basic Movements | **BM** | 10 | BM-01 ~ BM-10 |
| Facial Expressions | **FE** | 15 | FE-01 ~ FE-15 |
| Mouth Movements | **MM** | 8 | MM-01 ~ MM-08 |
| Number System | **NS** | 10 | NS-00 ~ NS-09 |
| Operation Symbols | **OS** | 15 | OS-01 ~ OS-15 |
| Digital Elements | **DE** | 10 | DE-01 ~ DE-10 |
| WIAverse Meta | **WM** | 15 | WM-01 ~ WM-15 |

### 전체 제스처 코드표

#### 1. 기본손모양 (Basic Hand Shapes)
```
BH-01  완성 (Completion / Fist)
BH-02  정밀 (Precision / Precision Grip)
BH-03  선택 (Selection / Hand Point)
BH-04  정중함 (Respect / Stop Hand)
BH-05  확장 (Expansion / Raised Hands)
BH-06  강조 (Emphasis / Wave)
BH-07  화합 (Harmony / Handshake)
BH-08  포용 (Embrace / Palms Up)
BH-09  연결 (Connection / Link)
BH-10  창조 (Creation / Art)
```

#### 2. 기본움직임 (Basic Movements)
```
BM-01  상승 (Rising / Upward)
BM-02  하강 (Descending / Downward)
BM-03  전진 (Forward / Advance)
BM-04  후퇴 (Backward / Retreat)
BM-05  확산 (Divergence / Expansion)
BM-06  수렴 (Convergence / Focus)
BM-07  순환 (Circulation / Cycle)
BM-08  파동 (Wave / Fluctuation)
BM-09  급변 (Sudden Change / Breakthrough)
BM-10  평온 (Stillness / Peace)
```

#### 3. 표정체계 (Facial Expressions)
```
FE-01  활짝웃음 (Broad Smile / Joy)
FE-02  미소 (Gentle Smile / Contentment)
FE-03  중립 (Neutral / Objectivity)
FE-04  집중 (Focus / Thinking)
FE-05  놀람 (Surprise / Astonishment)
FE-06  공감 (Empathy / Pleading)
FE-07  결의 (Determination / Resolve)
FE-08  평화 (Peace / Relieved)
FE-09  경외 (Awe / Hushed)
FE-10  감사 (Gratitude / Folded Hands)
FE-11  희망 (Hope / Star)
FE-12  지혜 (Wisdom / Mage)
FE-13  보호 (Protection / Shield)
FE-14  창조 (Creation / Art)
FE-15  완성 (Completion / Sparkles)
```

#### 4. 입움직임 (Mouth Movements)
```
MM-01  또렷발음 (Clear Articulation / Speaking)
MM-02  쉿소리 (Shush / Quiet)
MM-03  입술오므림 (Pursed Lips / Kissing)
MM-04  입벌림 (Open Mouth / Wonder)
MM-05  입술떨림 (Lip Tremor / Emotion)
MM-06  입모양변화 (Mouth Shape Change / Drama)
MM-07  호흡조절 (Breath Control / Mindfulness)
MM-08  침묵표현 (Silent Expression / Zipper)
```

#### 5. 숫자시스템 (Number System)
```
NS-00  0 (Zero / Circle / Infinity)
NS-01  1 (One / Unity)
NS-02  2 (Two / Balance)
NS-03  3 (Three / Trinity)
NS-04  4 (Four / Stability)
NS-05  5 (Five / Change)
NS-06  6 (Six / Harmony)
NS-07  7 (Seven / Mystery)
NS-08  8 (Eight / Infinity)
NS-09  9 (Nine / Completion)
```

#### 6. 연산기호 (Operation Symbols)
```
OS-01  더하기 (Addition / Plus)
OS-02  빼기 (Subtraction / Minus)
OS-03  곱하기 (Multiplication / Multiply)
OS-04  나누기 (Division / Divide)
OS-05  같다 (Equals / Balance)
OS-06  점 (Dot / Precision)
OS-07  슬래시 (Slash / Division)
OS-08  퍼센트 (Percent / Ratio)
OS-09  감탄 (Exclamation / Emphasis)
OS-10  의문 (Question / Inquiry)
OS-11  분류 (Category / Tag)
OS-12  연결 (Network / Connection)
OS-13  그리고 (And / Conjunction)
OS-14  대략 (Approximately / Wave)
OS-15  WIA포즈 (WIA Pose / Symbol)
```

#### 7. 디지털 요소 (Digital Elements)
```
DE-01  차원연결 (Dimension Link / Multi-dimensional)
DE-02  정밀선택 (Precision Select / Target)
DE-03  시공간회전 (Spacetime Rotation / Spiral)
DE-04  무한복제 (Infinite Clone / Cloning)
DE-05  투명모드 (Transparent Mode / Ghost)
DE-06  텔레파시 (Telepathy / Brain)
DE-07  현실변환 (Reality Shift / Transformation)
DE-08  에너지집중 (Energy Focus / Lightning)
DE-09  시간제어 (Time Control / Timer)
DE-10  완전조화 (Perfect Harmony / Yin Yang)
```

#### 8. 위아버스 특화 (WIAverse Meta)
```
WM-01  눈빛강도 (Gaze Intensity / Eye)
WM-02  공간이동 (Space Teleport / Globe)
WM-03  감정오라 (Emotion Aura / Sparkles)
WM-04  시간조절 (Time Dilation / Clock)
WM-05  기억공유 (Memory Share / Brain)
WM-06  에너지전달 (Energy Transfer / Lightning)
WM-07  차원확장 (Dimension Expand / Crystal Ball)
WM-08  진동동조 (Vibration Sync / Music)
WM-09  무한확장 (Infinite Expansion / Infinity)
WM-10  창조투영 (Creation Projection / Art)
WM-11  치유공명 (Healing Resonance / Green Heart)
WM-12  지혜전수 (Wisdom Transfer / Books)
WM-13  사랑증폭 (Love Amplify / Sparkling Heart)
WM-14  평화확산 (Peace Spread / Dove)
WM-15  완전소통 (Ultimate Communication / Galaxy)
```

---

## 🧩 코드 체계 B: 구성 요소 기반 (Component-Based)

### 형식
```
[손모양]-[위치]-[움직임]-[방향]-[비수지]
HS##-LC##-MV##-OR##-NM##
```

### 구성 요소 코드

#### 손 모양 (Handshape)
```
HS01  주먹 (Fist)
HS02  손바닥 (Flat Hand)
HS03  집게 (Precision Grip)
HS04  V자 (V-Shape)
HS05  삼지창 (Trident)
HS06  4개 손가락 (Four Fingers)
HS07  검지 (Index Finger)
HS08  원형 (Circle)
HS09  포용 (Cupped Hands)
HS10  합장 (Prayer)
HS11  연결 (Interlocked)
HS12  양손 펼침 (Both Hands Open)
HS13  악수 (Handshake)
HS14  십자 (Cross)
HS15  X자 (X-Shape)
HS16  평행선 (Parallel)
HS17  무한대 (Infinity)
HS18  나선 (Spiral)
```

#### 위치 (Location)
```
LC01  머리 위 (Above Head)
LC02  이마 (Forehead)
LC03  눈 (Eyes)
LC04  코 (Nose)
LC05  입 (Mouth)
LC06  턱 (Chin)
LC07  가슴 (Chest)
LC08  배 (Abdomen)
LC09  중립 공간 (Neutral Space)
LC10  좌측 (Left Side)
LC11  우측 (Right Side)
LC12  전방 공간 (Front Space)
```

#### 움직임 (Movement)
```
MV01  상승 (Up)
MV02  하강 (Down)
MV03  전진 (Forward)
MV04  후퇴 (Backward)
MV05  확산 (Diverge)
MV06  수렴 (Converge)
MV07  원형 (Circle)
MV08  파동 (Wave)
MV09  급변 (Sharp)
MV10  정지 (Still)
MV11  좌우 (Horizontal)
MV12  회전 (Rotate)
MV13  나선 (Spiral)
MV14  반복 (Repeat)
MV15  페이드 (Fade)
```

#### 방향 (Orientation)
```
OR01  전방 (Forward)
OR02  후방 (Backward)
OR03  상방 (Upward)
OR04  하방 (Downward)
OR05  측방-좌 (Left Side)
OR06  측방-우 (Right Side)
```

#### 비수지 신호 (Non-manual)
```
NM-FE01 ~ NM-FE15  표정 (Facial Expressions)
NM-MM01 ~ NM-MM08  입모양 (Mouth Movements)
```

### 구성 요소 기반 코드 예시

#### 예시 1: BH-01 (완성)
```
카테고리 코드:  BH-01
구성 요소 코드:  HS01-LC07-MV10-OR02-FE15
설명:           [주먹]-[가슴]-[정지]-[후방]-[완성표정]
```

#### 예시 2: BM-01 (상승)
```
카테고리 코드:  BM-01
구성 요소 코드:  HS02-LC09-MV01-OR03-FE11
설명:           [손바닥]-[중립공간]-[상승]-[상방]-[희망표정]
```

#### 예시 3: NS-02 (숫자 2)
```
카테고리 코드:  NS-02
구성 요소 코드:  HS04-LC09-MV10-OR02-FE03
설명:           [V자]-[중립공간]-[정지]-[후방]-[중립표정]
```

#### 예시 4: OS-01 (더하기)
```
카테고리 코드:  OS-01
구성 요소 코드:  HS14-LC09-MV10-OR02-FE03
설명:           [십자]-[중립공간]-[정지]-[후방]-[중립표정]
```

#### 예시 5: DE-06 (텔레파시)
```
카테고리 코드:  DE-06
구성 요소 코드:  HS02-LC02-MV07-OR03-FE04
설명:           [손바닥]-[이마]-[원형]-[상방]-[집중표정]
```

---

## 🔄 양방향 매핑 시스템

### 카테고리 → 구성 요소
```
BH-01 → HS01-LC07-MV10-OR02-FE15
```

### 구성 요소 → 카테고리
```
HS01-LC07-MV10-OR02-FE15 → BH-01
```

### 이점
- **간단한 참조**: 카테고리 코드 (BH-01)
- **상세한 분석**: 구성 요소 코드 (HS01-LC07-MV10-OR02-FE15)
- **AI 학습**: 구성 요소 분해로 패턴 학습
- **확장성**: 새로운 조합 생성 용이

---

## 📊 전체 93개 제스처 매핑표

### 기본손모양 (10개)
| 카테고리 | 구성 요소 코드 | 이름 |
|----------|----------------|------|
| BH-01 | HS01-LC07-MV10-OR02-FE15 | 완성 |
| BH-02 | HS03-LC09-MV10-OR03-FE03 | 정밀 |
| BH-03 | HS02-LC09-MV10-OR02-FE03 | 선택 |
| BH-04 | HS02-LC09-MV10-OR01-FE03 | 정중함 |
| BH-05 | HS12-LC01-MV10-OR01-FE01 | 확장 |
| BH-06 | HS02-LC09-MV11-OR02-FE02 | 강조 |
| BH-07 | HS13-LC09-MV10-OR05-FE03 | 화합 |
| BH-08 | HS09-LC07-MV10-OR03-FE02 | 포용 |
| BH-09 | HS11-LC07-MV10-OR02-FE03 | 연결 |
| BH-10 | HS02-LC09-MV12-OR02-FE14 | 창조 |

### 기본움직임 (10개)
| 카테고리 | 구성 요소 코드 | 이름 |
|----------|----------------|------|
| BM-01 | HS02-LC09-MV01-OR03-FE11 | 상승 |
| BM-02 | HS02-LC09-MV02-OR03-FE08 | 하강 |
| BM-03 | HS02-LC09-MV03-OR01-FE07 | 전진 |
| BM-04 | HS02-LC09-MV04-OR01-FE04 | 후퇴 |
| BM-05 | HS12-LC09-MV05-OR01-FE11 | 확산 |
| BM-06 | HS12-LC09-MV06-OR01-FE04 | 수렴 |
| BM-07 | HS02-LC09-MV07-OR02-FE03 | 순환 |
| BM-08 | HS02-LC09-MV08-OR02-FE03 | 파동 |
| BM-09 | HS02-LC09-MV09-OR02-FE07 | 급변 |
| BM-10 | HS02-LC07-MV10-OR03-FE08 | 평온 |

### 표정체계 (15개)
| 카테고리 | 구성 요소 코드 | 이름 |
|----------|----------------|------|
| FE-01 | HS00-LC00-MV00-OR00-FE01 | 활짝웃음 |
| FE-02 | HS00-LC00-MV00-OR00-FE02 | 미소 |
| FE-03 | HS00-LC00-MV00-OR00-FE03 | 중립 |
| FE-04 | HS00-LC00-MV00-OR00-FE04 | 집중 |
| FE-05 | HS00-LC00-MV00-OR00-FE05 | 놀람 |
| FE-06 | HS00-LC00-MV00-OR00-FE06 | 공감 |
| FE-07 | HS00-LC00-MV00-OR00-FE07 | 결의 |
| FE-08 | HS00-LC00-MV00-OR00-FE08 | 평화 |
| FE-09 | HS00-LC00-MV00-OR00-FE09 | 경외 |
| FE-10 | HS00-LC00-MV00-OR00-FE10 | 감사 |
| FE-11 | HS00-LC00-MV00-OR00-FE11 | 희망 |
| FE-12 | HS00-LC00-MV00-OR00-FE12 | 지혜 |
| FE-13 | HS00-LC00-MV00-OR00-FE13 | 보호 |
| FE-14 | HS00-LC00-MV00-OR00-FE14 | 창조 |
| FE-15 | HS00-LC00-MV00-OR00-FE15 | 완성 |

*참고: 표정은 손 사용 없음 (HS00-LC00-MV00-OR00)*

### 입움직임 (8개)
| 카테고리 | 구성 요소 코드 | 이름 |
|----------|----------------|------|
| MM-01 | HS00-LC05-MV00-OR00-MM01 | 또렷발음 |
| MM-02 | HS00-LC05-MV00-OR00-MM02 | 쉿소리 |
| MM-03 | HS00-LC05-MV00-OR00-MM03 | 입술오므림 |
| MM-04 | HS00-LC05-MV00-OR00-MM04 | 입벌림 |
| MM-05 | HS00-LC05-MV00-OR00-MM05 | 입술떨림 |
| MM-06 | HS00-LC05-MV00-OR00-MM06 | 입모양변화 |
| MM-07 | HS00-LC05-MV00-OR00-MM07 | 호흡조절 |
| MM-08 | HS00-LC05-MV00-OR00-MM08 | 침묵표현 |

### 숫자시스템 (10개)
| 카테고리 | 구성 요소 코드 | 이름 |
|----------|----------------|------|
| NS-00 | HS08-LC09-MV10-OR02-FE03 | 0 |
| NS-01 | HS02-LC09-MV10-OR02-FE03 | 1 |
| NS-02 | HS04-LC09-MV10-OR02-FE03 | 2 |
| NS-03 | HS05-LC09-MV10-OR02-FE03 | 3 |
| NS-04 | HS06-LC09-MV10-OR02-FE03 | 4 |
| NS-05 | HS02-LC09-MV10-OR02-FE03 | 5 |
| NS-06 | HS09-LC07-MV10-OR03-FE03 | 6 |
| NS-07 | HS10-LC07-MV10-OR01-FE03 | 7 |
| NS-08 | HS17-LC09-MV07-OR02-FE03 | 8 |
| NS-09 | HS02-LC09-MV07-OR02-FE03 | 9 |

### 연산기호 (15개)
| 카테고리 | 구성 요소 코드 | 이름 |
|----------|----------------|------|
| OS-01 | HS14-LC09-MV10-OR02-FE03 | 더하기 |
| OS-02 | HS02-LC09-MV10-OR04-FE03 | 빼기 |
| OS-03 | HS15-LC09-MV10-OR02-FE03 | 곱하기 |
| OS-04 | HS16-LC09-MV10-OR02-FE03 | 나누기 |
| OS-05 | HS16-LC09-MV10-OR02-FE03 | 같다 |
| OS-06 | HS07-LC09-MV10-OR02-FE03 | 점 |
| OS-07 | HS02-LC09-MV03-OR02-FE03 | 슬래시 |
| OS-08 | HS08-LC09-MV03-OR02-FE03 | 퍼센트 |
| OS-09 | HS07-LC09-MV01-OR02-FE09 | 감탄 |
| OS-10 | HS07-LC09-MV08-OR02-FE04 | 의문 |
| OS-11 | HS02-LC09-MV07-OR02-FE03 | 분류 |
| OS-12 | HS11-LC09-MV05-OR02-FE03 | 연결 |
| OS-13 | HS13-LC09-MV10-OR05-FE02 | 그리고 |
| OS-14 | HS02-LC09-MV08-OR02-FE03 | 대략 |
| OS-15 | HS12-LC01-MV10-OR01-FE01 | WIA포즈 |

### 디지털 요소 (10개)
| 카테고리 | 구성 요소 코드 | 이름 |
|----------|----------------|------|
| DE-01 | HS11-LC12-MV05-OR02-FE04 | 차원연결 |
| DE-02 | HS07-LC09-MV10-OR02-FE04 | 정밀선택 |
| DE-03 | HS18-LC09-MV13-OR02-FE04 | 시공간회전 |
| DE-04 | HS17-LC09-MV07-OR02-FE03 | 무한복제 |
| DE-05 | HS02-LC09-MV15-OR03-FE03 | 투명모드 |
| DE-06 | HS02-LC02-MV07-OR03-FE04 | 텔레파시 |
| DE-07 | HS02-LC09-MV12-OR02-FE04 | 현실변환 |
| DE-08 | HS09-LC09-MV06-OR03-FE07 | 에너지집중 |
| DE-09 | HS02-LC09-MV07-OR02-FE04 | 시간제어 |
| DE-10 | HS17-LC09-MV07-OR02-FE08 | 완전조화 |

### 위아버스 특화 (15개)
| 카테고리 | 구성 요소 코드 | 이름 |
|----------|----------------|------|
| WM-01 | HS00-LC03-MV00-OR00-FE04 | 눈빛강도 |
| WM-02 | HS02-LC09-MV03-OR01-FE11 | 공간이동 |
| WM-03 | HS12-LC09-MV05-OR01-FE01 | 감정오라 |
| WM-04 | HS02-LC09-MV07-OR02-FE04 | 시간조절 |
| WM-05 | HS02-LC02-MV03-OR01-FE12 | 기억공유 |
| WM-06 | HS02-LC09-MV03-OR01-FE07 | 에너지전달 |
| WM-07 | HS09-LC09-MV05-OR03-FE09 | 차원확장 |
| WM-08 | HS02-LC09-MV08-OR02-FE02 | 진동동조 |
| WM-09 | HS17-LC09-MV05-OR02-FE11 | 무한확장 |
| WM-10 | HS02-LC09-MV03-OR01-FE14 | 창조투영 |
| WM-11 | HS09-LC07-MV08-OR03-FE08 | 치유공명 |
| WM-12 | HS02-LC02-MV03-OR01-FE12 | 지혜전수 |
| WM-13 | HS09-LC07-MV05-OR03-FE01 | 사랑증폭 |
| WM-14 | HS12-LC09-MV05-OR01-FE08 | 평화확산 |
| WM-15 | HS12-LC09-MV05-OR01-FE15 | 완전소통 |

---

## 🎯 코드 사용 가이드

### 일반 사용자
**카테고리 코드 사용 권장**
```
"BH-01을 실행하세요" (완성)
"NS-05를 사용하세요" (숫자 5)
```

### 개발자/연구자
**구성 요소 코드 사용 권장**
```python
gesture = "HS01-LC07-MV10-OR02-FE15"
components = parse_gesture(gesture)
# {handshape: "HS01", location: "LC07", ...}
```

### AI 학습
**구성 요소 벡터화**
```python
# 벡터 표현
gesture_vector = [
    handshape_embedding(HS01),   # [0.2, 0.5, ...]
    location_embedding(LC07),    # [0.8, 0.1, ...]
    movement_embedding(MV10),    # [0.0, 0.0, ...]
    orientation_embedding(OR02), # [0.9, 0.2, ...]
    nonmanual_embedding(FE15)    # [0.6, 0.7, ...]
]
```

---

## 🔮 확장 계획

### Phase 2: 조합 제스처 (Compound Gestures)
```
형식: [제스처1]+[제스처2]
예시: BH-01+FE-01 (완성 + 활짝웃음)
코드: CPD-001
```

### Phase 3: 시퀀스 제스처 (Sequential Gestures)
```
형식: [제스처1]→[제스처2]→[제스처3]
예시: BM-01→BM-05→FE-11 (상승→확산→희망)
코드: SEQ-001
```

### Phase 4: 문맥 제스처 (Contextual Gestures)
```
형식: [제스처]@[맥락]
예시: BH-07@business (화합@비즈니스)
코드: CTX-001
```

### Phase 5: 문화 변형 (Cultural Variants)
```
형식: [제스처]#[문화코드]
예시: BH-10#KR (창조#한국)
코드: VAR-001
```

---

## 📚 비교: WIA Braille vs WIA Talk

### WIA Braille
```
IPA 음소 → 점자 패턴
예: /p/ → ⠏ (1-2-3-4)
     /b/ → ⠃ (1-2)
     /t/ → ⠞ (2-3-4-5)
```

### WIA Talk
```
제스처 → 코드
예: 완성 → BH-01 → HS01-LC07-MV10-OR02-FE15
    숫자2 → NS-02 → HS04-LC09-MV10-OR02-FE03
    더하기 → OS-01 → HS14-LC09-MV10-OR02-FE03
```

### 공통점
- ✅ 구성 요소 분해
- ✅ 체계적 코드화
- ✅ 확장 가능한 구조
- ✅ 기계 학습 친화적
- ✅ 보편성 추구

---

## 💻 구현 예시

### JSON 형식
```json
{
  "gesture_id": "BH-01",
  "name": {
    "ko": "완성",
    "en": "Completion"
  },
  "category": "basic-hand-shapes",
  "components": {
    "handshape": "HS01",
    "location": "LC07",
    "movement": "MV10",
    "orientation": "OR02",
    "nonmanual": "FE15"
  },
  "code": "HS01-LC07-MV10-OR02-FE15",
  "emoji": "✊",
  "description": {
    "ko": "자연스러운 주먹",
    "en": "Natural fist"
  }
}
```

### Python 클래스
```python
class Gesture:
    def __init__(self, category_code, component_code):
        self.category_code = category_code  # "BH-01"
        self.component_code = component_code  # "HS01-LC07-MV10-OR02-FE15"
        self.components = self.parse_components(component_code)

    def parse_components(self, code):
        parts = code.split('-')
        return {
            'handshape': parts[0],
            'location': parts[1],
            'movement': parts[2],
            'orientation': parts[3],
            'nonmanual': parts[4]
        }

    def to_vector(self):
        return [
            self.embed(self.components['handshape']),
            self.embed(self.components['location']),
            self.embed(self.components['movement']),
            self.embed(self.components['orientation']),
            self.embed(self.components['nonmanual'])
        ]
```

### SQL 데이터베이스
```sql
CREATE TABLE gestures (
    id INTEGER PRIMARY KEY,
    category_code VARCHAR(10),
    component_code VARCHAR(50),
    name_ko VARCHAR(100),
    name_en VARCHAR(100),
    handshape VARCHAR(10),
    location VARCHAR(10),
    movement VARCHAR(10),
    orientation VARCHAR(10),
    nonmanual VARCHAR(10),
    emoji VARCHAR(10),
    created_at TIMESTAMP
);

-- 예시 삽입
INSERT INTO gestures VALUES (
    1,
    'BH-01',
    'HS01-LC07-MV10-OR02-FE15',
    '완성',
    'Completion',
    'HS01',
    'LC07',
    'MV10',
    'OR02',
    'FE15',
    '✊',
    NOW()
);
```

---

## 🎓 학습 순서

### 1단계: 카테고리 코드 익히기 (1주)
```
BH-01 ~ BH-10  (기본손모양 10개)
BM-01 ~ BM-10  (기본움직임 10개)
NS-00 ~ NS-09  (숫자 10개)
```

### 2단계: 구성 요소 이해하기 (2주)
```
HS01~18  (손 모양 18종)
LC01~12  (위치 12종)
MV01~15  (움직임 15종)
```

### 3단계: 조합 원리 파악하기 (1개월)
```
카테고리 코드 → 구성 요소 분해
구성 요소 조합 → 새 제스처 생성
```

### 4단계: 고급 활용 (3개월)
```
조합 제스처
시퀀스 제스처
문맥 이해
```

---

## 🌟 핵심 가치

### WIA Braille이 이룬 것
✨ IPA → 점자 매핑으로 **7,000개 언어** 해결

### WIA Talk이 이루는 것
✨ 제스처 → 코드 매핑으로 **300+ 수화 언어** 통합
✨ 구성 요소 기반으로 **무한 확장** 가능
✨ AI 학습으로 **자동 인식** 실현

---

**홍익인간 (弘益人間) - Benefit All Humanity**

*93개 제스처에서 시작하여 무한한 소통으로*

---

**작성일**: 2025-12-08
**작성자**: Claude (Anthropic) + WIA Team
**버전**: 1.0.0

---

## 🔒 안전 기준 (2025-12-13 추가)

### 지문 보호 원칙
모든 WIA Talk 제스처는 **지문이 보이지 않는 방향**으로 설계됩니다.

| 방향 | 허용 | 설명 |
|------|------|------|
| 손등 방향 | ✅ | 기본 권장 |
| 측면 45도 | ✅ | 손등 일부 보임 |
| 손바닥 전체 | ⚠️ | 라인아트로 지문 생략 |

### 문화적 안전 원칙

| 제스처 | 대응 |
|--------|------|
| V-Sign | 손바닥 방향 (영국/호주 안전) |
| OK Sign | "완성/동그라미" 중립 명명 |

상세 기준: `WIA-TALK-SAFETY-STANDARD.md` 참조

