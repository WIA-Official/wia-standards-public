# WIA-RUST-LEARN Manim Animations

## 🎬 공식 유튜브 채널

<div align="center">

### [**@yeonsamheum**](https://www.youtube.com/@yeonsamheum)

[![YouTube Channel](https://img.shields.io/badge/YouTube-@yeonsamheum-FF0000?style=for-the-badge&logo=youtube)](https://www.youtube.com/@yeonsamheum)

**28편 전체 시리즈를 공식 채널에서 시청하세요!**

</div>

---

## 📚 WIA-RUST-LEARN 전체 커리큘럼

Rust 프로그래밍을 시각적 애니메이션으로 배우는 28편의 완전한 교육 시리즈입니다.

### 🎯 핵심 철학

> **"Rust는 어렵지 않다. 다를 뿐이다."**

### 🔗 NRT: Non-Repeatable Transfer

NFT가 디지털 자산의 유일한 소유권을 증명하듯,
Rust의 소유권 시스템은 메모리 데이터의 유일한 소유자를 컴파일 타임에 보장합니다.

---

## 📊 28편 에피소드 구성

### 🔷 Core Certificate (Level 0-5: 19편)

| Level | 주제 | 에피소드 | 파일 |
|:-----:|------|:--------:|------|
| 0 | 시작하기 | EP 0-01 ~ 0-03 | `level_0_setup.py` |
| 1 | 기초 | EP 1-01 ~ 1-04 | `level_1_basics.py` |
| 2 | 소유권 (NRT!) | EP 2-01 ~ 2-04 | `level_2_ownership.py` |
| 3 | 구조화 | EP 3-01 ~ 3-03 | `level_3_structure.py` |
| 4 | 고급 기능 | EP 4-01 ~ 4-03 | `level_4_advanced.py` |
| 5 | 동시성 | EP 5-01 ~ 5-02 | `level_5_concurrency.py` |

### 🔮 Expert Certificate (Level 6-8: 6편)

| Level | 주제 | 에피소드 | 파일 |
|:-----:|------|:--------:|------|
| 6 | 테스팅 & 도구 | EP 6-01 ~ 6-02 | `level_6_testing.py` |
| 7 | 고급 패턴 | EP 7-01 ~ 7-02 | `level_7_patterns.py` |
| 8 | Unsafe & 매크로 | EP 8-01 ~ 8-02 | `level_8_unsafe.py` |

### 👑 Master Certificate (Level 9: 3편)

| Level | 주제 | 에피소드 | 파일 |
|:-----:|------|:--------:|------|
| 9 | 실전 프로젝트 | EP 9-01 ~ 9-03 | `level_9_projects.py` |

---

## 🛠️ 설치 및 사용법

### 1. Manim 설치

```bash
pip install manim
```

### 2. 에피소드 렌더링

```bash
cd wia-rust-learn/manim/episodes

# 개별 에피소드 (저화질 - 빠른 미리보기)
manim -pql level_2_ownership.py EP_2_01_OwnershipRules

# 개별 에피소드 (고화질 - YouTube용)
manim -pqh level_2_ownership.py EP_2_01_OwnershipRules

# 레벨 전체 렌더링
manim -pqh level_2_ownership.py --all
```

### 3. 품질 옵션

| 옵션 | 해상도 | 용도 |
|------|--------|------|
| `-ql` | 480p 15fps | 빠른 테스트 |
| `-qm` | 720p 30fps | 미리보기 |
| `-qh` | 1080p 60fps | **YouTube 권장** |
| `-qk` | 4K 60fps | 고품질 |

### 4. 출력 위치

```
media/videos/[파일명]/[품질]/[Scene이름].mp4
```

---

## 🎙️ 영상 완성하기

### 음성 추가 옵션

1. **직접 녹음**: 스크립트의 한국어 텍스트 활용
2. **OpenAI TTS**: `tts-1-hd` 모델 사용
3. **ElevenLabs**: 자연스러운 한국어 음성
4. **Coqui TTS**: 오픈소스 대안

### 편집 도구

- **DaVinci Resolve** (무료): 전문 편집
- **CapCut**: 간단한 편집 & 자막
- **FFmpeg**: 배치 처리

---

## 📁 파일 구조

```
wia-rust-learn/manim/
├── README.md                 # 이 파일
├── ownership_animation.py    # NRT 컨셉 데모
└── episodes/
    ├── level_0_setup.py      # Level 0: 시작하기
    ├── level_1_basics.py     # Level 1: 기초
    ├── level_2_ownership.py  # Level 2: 소유권 ⭐
    ├── level_3_structure.py  # Level 3: 구조화
    ├── level_4_advanced.py   # Level 4: 고급 기능
    ├── level_5_concurrency.py # Level 5: 동시성
    ├── level_6_testing.py    # Level 6: 테스팅
    ├── level_7_patterns.py   # Level 7: 고급 패턴
    ├── level_8_unsafe.py     # Level 8: Unsafe & 매크로
    └── level_9_projects.py   # Level 9: 실전 프로젝트
```

---

## 🎨 커스터마이징

### 테마 색상

```python
RUST_ORANGE = "#F74C00"    # Rust 공식 오렌지
STACK_GREEN = "#2ECC71"    # Stack 메모리
HEAP_PURPLE = "#9B59B6"    # Heap 메모리
MEMORY_BLUE = "#4A90D9"    # 참조/포인터
WARNING_YELLOW = "#F39C12" # 경고/주의
ERROR_RED = "#E74C3C"      # 에러/패닉
SUCCESS_GREEN = "#27AE60"  # 성공/완료
```

### 배경

```python
DARK_BG = "#1a1a2e"  # 어두운 배경 (기본)
```

---

## 📺 YouTube 업로드 체크리스트

- [ ] 1080p 또는 4K로 렌더링
- [ ] 음성/나레이션 추가
- [ ] 썸네일 제작 (1280x720)
- [ ] 제목: `[WIA-RUST-LEARN] EP X-XX: 제목`
- [ ] 설명: 에피소드 내용 + 타임스탬프
- [ ] 태그: Rust, 러스트, 프로그래밍, 소유권, WIA
- [ ] 재생목록 추가: WIA-RUST-LEARN 시리즈

---

## 📖 책 참조

이 영상 시리즈의 내용은 다음 책에서 상세히 다룹니다:

> **WIA-RUST-LEARN** (출간 예정)
>
> ISBN: (발급 예정)
>
> 영상과 책을 함께 보면 더욱 효과적입니다!

---

## 🔗 관련 링크

- 📺 **YouTube**: [@yeonsamheum](https://www.youtube.com/@yeonsamheum)
- 📚 **GitHub**: [WIA-Official/wia-standards](https://github.com/WIA-Official/wia-standards)
- 🌐 **WIA**: World Certification Industry Association

---

© 2025 SmileStory Inc. / WIA

**弘益人間 (홍익인간)** - 널리 인간을 이롭게 하라
