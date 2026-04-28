# 🦀 WIA-RUST-LEARN 완전 가이드

## 📺 공식 유튜브 채널
### **https://www.youtube.com/@yeonsamheum**

---

## 🎯 이 가이드의 목적

Claude Code가 만들어준 **28편의 Manim 애니메이션 스크립트**를
**실제 YouTube 영상으로 변환**하는 전체 과정을 안내합니다.

---

## 📦 무엇이 만들어졌나요?

### 전체 파일 목록

```
wia-rust-learn/
├── manim/
│   ├── README.md              # 📖 Manim 사용 설명서
│   ├── ownership_animation.py # 🔗 NRT 데모 영상
│   └── episodes/              # 📚 28편 에피소드
│       ├── level_0_setup.py       # 3편
│       ├── level_1_basics.py      # 4편
│       ├── level_2_ownership.py   # 4편 (⭐ NRT!)
│       ├── level_3_structure.py   # 3편
│       ├── level_4_advanced.py    # 3편
│       ├── level_5_concurrency.py # 2편
│       ├── level_6_testing.py     # 2편
│       ├── level_7_patterns.py    # 2편
│       ├── level_8_unsafe.py      # 2편
│       └── level_9_projects.py    # 3편
├── curriculum/index.html     # 🎓 커리큘럼 웹페이지
├── videos/index.html         # 📺 비디오 목록 웹페이지
├── certification/index.html  # 🏆 인증 시스템 웹페이지
├── ownership/index.html      # 🔗 소유권 시각화 웹페이지
└── GUIDE.md                  # 📘 이 가이드
```

### 28편 에피소드 전체 목록

| EP | 제목 | 레벨 | 인증 |
|----|------|:----:|:----:|
| 0-01 | Why Rust? | 0 | 🔷 |
| 0-02 | Installing Rust | 0 | 🔷 |
| 0-03 | Cargo Deep Dive | 0 | 🔷 |
| 1-01 | Variables & Types | 1 | 🔷 |
| 1-02 | Functions & Control Flow | 1 | 🔷 |
| 1-03 | String vs &str | 1 | 🔷 |
| 1-04 | Collections & Iterators | 1 | 🔷 |
| **2-01** | **Ownership Rules (NRT!)** | 2 | 🔷 |
| 2-02 | Move, Copy, Clone | 2 | 🔷 |
| 2-03 | References & Borrowing | 2 | 🔷 |
| 2-04 | Lifetimes | 2 | 🔷 |
| 3-01 | Structs & Enums | 3 | 🔷 |
| 3-02 | Error Handling | 3 | 🔷 |
| 3-03 | Modules & Crates | 3 | 🔷 |
| 4-01 | Generics & Traits | 4 | 🔷 |
| 4-02 | Closures | 4 | 🔷 |
| 4-03 | Smart Pointers | 4 | 🔷 |
| 5-01 | Threads & Channels | 5 | 🔷 |
| 5-02 | Async/Await | 5 | 🔷 |
| 6-01 | Testing | 6 | 🔮 |
| 6-02 | Cargo Advanced | 6 | 🔮 |
| 7-01 | Pattern Matching Deep | 7 | 🔮 |
| 7-02 | OOP in Rust | 7 | 🔮 |
| 8-01 | Unsafe Rust | 8 | 🔮 |
| 8-02 | Macros | 8 | 🔮 |
| 9-01 | CLI App (minigrep) | 9 | 👑 |
| 9-02 | Web API (Axum) | 9 | 👑 |
| 9-03 | System Tool (File Watcher) | 9 | 👑 |

---

## 🚀 Step-by-Step 영상 제작 가이드

### Step 1: 환경 설정

```bash
# Python 3.8+ 필요
python --version

# Manim 설치
pip install manim

# 프로젝트 폴더로 이동
cd /path/to/wia-standards/wia-rust-learn/manim/episodes
```

### Step 2: 테스트 렌더링 (1개 에피소드)

```bash
# 저화질로 빠르게 테스트 (30초 정도)
manim -pql level_0_setup.py EP_0_01_WhyRust

# -p: 렌더링 후 자동 재생
# -q: 품질 (l=low, m=medium, h=high, k=4K)
# -l: low quality (480p)
```

**결과 확인**: `media/videos/level_0_setup/480p15/EP_0_01_WhyRust.mp4`

### Step 3: 고화질 렌더링 (YouTube용)

```bash
# 1080p 60fps (YouTube 권장)
manim -qh level_0_setup.py EP_0_01_WhyRust

# 4K 렌더링 (고사양 PC 필요)
manim -qk level_0_setup.py EP_0_01_WhyRust
```

**결과**: `media/videos/level_0_setup/1080p60/EP_0_01_WhyRust.mp4`

### Step 4: 전체 레벨 일괄 렌더링

```bash
# Level 2 전체 (4개 에피소드)
manim -qh level_2_ownership.py --all

# 모든 레벨 순차 렌더링 (시간 오래 걸림!)
for f in level_*.py; do manim -qh $f --all; done
```

### Step 5: 음성 추가 (선택)

**옵션 A: 직접 녹음**
- 스크립트 안의 한국어 텍스트를 읽고 녹음
- Audacity (무료)로 편집

**옵션 B: AI TTS**
```python
# OpenAI TTS 예시
from openai import OpenAI
client = OpenAI()

response = client.audio.speech.create(
    model="tts-1-hd",
    voice="nova",
    input="Rust의 소유권 시스템을 알아봅시다."
)
response.stream_to_file("narration.mp3")
```

**옵션 C: 자막만**
- 나레이션 없이 자막으로만 설명
- CapCut 앱에서 자동 자막 생성

### Step 6: 영상 편집

**무료 도구:**
- **DaVinci Resolve**: 전문 편집 (무료!)
- **CapCut**: 간단한 편집 & 자막
- **Kdenlive**: 리눅스 오픈소스

**작업 내용:**
1. Manim 영상 + 나레이션 오디오 합치기
2. 인트로/아웃로 추가
3. 자막 삽입
4. 배경 음악 (저작권 주의!)

### Step 7: YouTube 업로드

**제목 형식:**
```
[WIA-RUST-LEARN] EP 2-01: Ownership Rules - NRT 개념 | Rust 프로그래밍
```

**설명 템플릿:**
```markdown
🦀 WIA-RUST-LEARN EP 2-01: Ownership Rules

Rust의 핵심인 소유권(Ownership) 시스템을 배웁니다.
NRT(Non-Repeatable Transfer) 개념으로 쉽게 이해해보세요!

📚 타임스탬프
0:00 인트로
0:30 소유권이란?
2:00 세 가지 규칙
5:00 NRT 개념
8:00 정리

🔗 관련 링크
- GitHub: https://github.com/WIA-Official/wia-standards
- 재생목록: (링크)

#Rust #러스트 #프로그래밍 #소유권 #WIA
```

**태그:**
```
Rust, 러스트, Rust 강좌, Rust 튜토리얼, Rust 입문,
소유권, Ownership, 시스템 프로그래밍, WIA,
프로그래밍 강좌, 코딩, 개발
```

**썸네일:**
- 크기: 1280 x 720
- 텍스트: 에피소드 번호 + 핵심 키워드
- 색상: Rust 오렌지 (#F74C00)

---

## 💡 활용 팁

### 각 에피소드 확인하기

```bash
# 파일 내용 확인
python level_2_ownership.py

# 출력:
# ╔══════════════════════════════════════════╗
# ║  EP 2-01: Ownership Rules                ║
# ║    manim -pql ... EP_2_01_OwnershipRules ║
# ║  ...                                     ║
# ╚══════════════════════════════════════════╝
```

### 색상 커스터마이징

각 파일 상단에서 색상 수정 가능:
```python
RUST_ORANGE = "#F74C00"  # 원하는 색으로 변경
DARK_BG = "#1a1a2e"      # 배경색 변경
```

### 텍스트 수정

한국어 텍스트는 코드 안에 있습니다:
```python
Text("Rust는 어렵지 않다", font_size=32)
# ↓ 변경
Text("Rust is not hard", font_size=32)
```

### 애니메이션 속도 조절

```python
self.wait(1)    # 1초 대기 → 0.5로 빠르게
self.play(..., run_time=0.5)  # 애니메이션 속도
```

---

## 🔧 문제 해결

### "No module named 'manim'"
```bash
pip install manim
# 또는
pip3 install manim
```

### 한글 깨짐
```bash
# 한글 폰트 설치 (Ubuntu)
sudo apt install fonts-noto-cjk

# macOS는 기본 지원
```

### 렌더링 너무 느림
```bash
# 저화질로 먼저 테스트
manim -ql file.py Scene

# GPU 가속 (있다면)
manim -ql --renderer=opengl file.py Scene
```

### 메모리 부족
- 4K 렌더링은 16GB+ RAM 권장
- 1080p는 8GB로 충분

---

## 📊 권장 워크플로우

```
1일차: Level 0 (3편) 렌더링 + 편집 + 업로드
2일차: Level 1 (4편) 렌더링 + 편집 + 업로드
3일차: Level 2 (4편) ⭐ 소유권! 가장 중요
4일차: Level 3 (3편)
5일차: Level 4 (3편)
6일차: Level 5 (2편) + Level 6 (2편)
7일차: Level 7 (2편) + Level 8 (2편)
8일차: Level 9 (3편) 🎉 완료!
```

**총 소요 시간**: 약 1-2주 (하루 3-4시간 기준)

---

## 📞 다음 단계

1. **먼저 EP 0-01 하나만 완성해서 업로드**
2. 반응 보고 나머지 진행
3. 책 출간과 연계
4. 커뮤니티 형성

---

## 🎬 유튜브 채널 정보

**채널명**: @yeonsamheum
**URL**: https://www.youtube.com/@yeonsamheum

**재생목록 구성 제안:**
1. WIA-RUST-LEARN Core (Level 0-5)
2. WIA-RUST-LEARN Expert (Level 6-8)
3. WIA-RUST-LEARN Master (Level 9)
4. WIA-RUST-LEARN 하이라이트

---

## 🙏 마무리

**"Rust는 어렵지 않다. 다를 뿐이다."**

이 28편의 영상으로 한국에서 Rust를 배우는 방식을 바꿔보세요.

**弘益人間** - 널리 인간을 이롭게 하라

---

© 2025 SmileStory Inc. / WIA
