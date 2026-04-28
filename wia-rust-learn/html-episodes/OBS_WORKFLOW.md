# OBS 촬영 워크플로우

> WIA Rust 84편 (Learn 28 + Intermediate 28 + Advanced 28) 영상 촬영용
> YouTube @yeonsamheum 채널

---

## 1. OBS 기본 설정

### 출력 설정 (Settings → Output)
```
출력 모드: 고급
인코더: x264 (또는 NVENC H.264)
비트레이트: 8,000 Kbps
키프레임 간격: 2초
프리셋: Quality
프로파일: High
```

### 비디오 설정 (Settings → Video)
```
기본 해상도: 1920x1080
출력 해상도: 1920x1080
FPS: 30
```

### 오디오 설정 (Settings → Audio)
```
샘플 레이트: 48kHz
채널: 스테레오
마이크: 사용 중인 마이크 선택
```

---

## 2. 씬(Scene) 구성

### Scene 1: 인트로 (5초)
```
소스:
- 이미지: WIA 로고 배경
- 텍스트: "WIA Rust Learn EP01: Why Rust?"
```

### Scene 2: 슬라이드 (메인)
```
소스:
- 브라우저: 슬라이드 HTML (전체화면)
- 오디오 입력 캡처: 마이크
- (선택) 마우스 하이라이터 플러그인
```

### Scene 3: 아웃트로 (5초)
```
소스:
- 이미지: 구독 유도 배경
- 텍스트: "구독과 좋아요 부탁드려요!"
```

---

## 3. 브라우저 소스 설정

### 슬라이드 URL 패턴
```
file:///home/user/wia-standards/wia-rust-learn/html-episodes/EP_X_XX_Title.html
```

### 브라우저 소스 속성
```
너비: 1920
높이: 1080
FPS: 30
CSS: (비워두기)
페이지 권한: 고급 액세스 허용
```

---

## 4. 촬영 체크리스트

### 촬영 전 (5분)
- [ ] OBS 실행 및 씬 확인
- [ ] 마이크 테스트 (레벨 -12dB ~ -6dB)
- [ ] 슬라이드 URL 로드 확인
- [ ] 저장 폴더 확인 (`~/Videos/WIA-Rust/`)
- [ ] 방해금지 모드 ON
- [ ] 물 준비

### 촬영 중
- [ ] 인트로 씬 5초 → 슬라이드 씬 전환
- [ ] NARRATION_SCRIPTS.md 참고하며 진행
- [ ] 슬라이드 넘기기: 스페이스바 또는 화살표
- [ ] 실수 시: 3초 멈춤 → 해당 부분 다시 말하기 (편집에서 컷)
- [ ] 마지막: 아웃트로 씬 5초

### 촬영 후
- [ ] 녹화 정지
- [ ] 파일명 변경 (아래 규칙)
- [ ] 다음 에피소드 슬라이드 로드

---

## 5. 파일 저장 규칙

### 파일명 형식
```
EP{레벨}-{번호}-{제목}.mp4

예시:
EP0-01-Why_Rust.mp4
EP2-03-Borrowing.mp4
EP9-03-System_Tool.mp4
```

### 폴더 구조
```
~/Videos/WIA-Rust/
├── Learn/
│   ├── EP0-01-Why_Rust.mp4
│   ├── EP0-02-Installing_Rust.mp4
│   └── ...
├── Intermediate/
│   └── ...
└── Advanced/
    └── ...
```

---

## 6. 예상 촬영 시간

| 에피소드 유형 | 슬라이드 수 | 촬영 시간 | 최종 영상 |
|--------------|------------|----------|----------|
| 기본 | 8-10개 | 6-8분 | 4-6분 |
| 긴 편 | 12-15개 | 10-12분 | 7-9분 |
| 실습 편 | 15-20개 | 15-20분 | 10-15분 |

**전체 예상:**
- Learn 28편: 약 4시간
- Intermediate 28편: 약 4시간
- Advanced 28편: 약 4시간
- **총 12시간** (촬영 기준, 편집 별도)

---

## 7. 실수 대처법

### 말 실수
```
❌ 하지 마세요: 녹화 중단 후 처음부터 다시
✅ 하세요: 3초 침묵 → "다시 말씀드리면..." → 이어서 진행
(편집에서 3초 침묵 구간 컷)
```

### 기술 문제
```
슬라이드 안 넘어감 → 브라우저 새로고침
소리 안 들림 → OBS 오디오 믹서 확인
```

---

## 8. 슬라이드 URL 목록 (Learn 28편)

```
# Level 0: 시작하기
EP_0_01_Why_Rust.html
EP_0_02_Installing_Rust.html
EP_0_03_Cargo_Deep_Dive.html

# Level 1: 기초 문법
EP_1_01_Variables_Types.html
EP_1_02_Functions_Control.html
EP_1_03_Strings.html
EP_1_04_Collections.html

# Level 2: 소유권
EP_2_01_Ownership_Rules.html
EP_2_02_Move_Copy_Clone.html
EP_2_03_Borrowing.html
EP_2_04_Lifetimes.html

# Level 3: 구조화
EP_3_01_Structs_Enums.html
EP_3_02_Error_Handling.html
EP_3_03_Modules_Crates.html

# Level 4: 제네릭과 트레이트
EP_4_01_Generics_Traits.html
EP_4_02_Closures.html
EP_4_03_Smart_Pointers.html

# Level 5: 동시성
EP_5_01_Threads_Channels.html
EP_5_02_Async_Await.html

# Level 6: 테스트와 도구
EP_6_01_Testing.html
EP_6_02_Cargo_Advanced.html

# Level 7: 고급 패턴
EP_7_01_Pattern_Matching.html
EP_7_02_OOP_Rust.html

# Level 8: 시스템
EP_8_01_Unsafe_Rust.html
EP_8_02_Macros.html

# Level 9: 프로젝트
EP_9_01_CLI_App.html
EP_9_02_Web_API.html
EP_9_03_System_Tool.html
```

---

## 9. 단축키

| 동작 | 단축키 |
|------|--------|
| 녹화 시작/중지 | Ctrl + Shift + R |
| 씬 1 (인트로) | Ctrl + 1 |
| 씬 2 (슬라이드) | Ctrl + 2 |
| 씬 3 (아웃트로) | Ctrl + 3 |
| 슬라이드 다음 | Space / → |
| 슬라이드 이전 | ← |

---

**다음 단계:** UPLOAD_CHECKLIST.md → 편집 및 업로드

© 2025 SmileStory Inc. / WIA
