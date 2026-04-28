# YouTube 업로드 체크리스트

> WIA Rust 84편 업로드용
> YouTube @yeonsamheum 채널

---

## 1. 편집 워크플로우

### 기본 편집 (DaVinci Resolve / Premiere)
```
1. 원본 영상 불러오기
2. 3초 침묵 구간 찾아서 컷
3. 인트로 (5초) 확인
4. 아웃트로 (5초) 확인
5. 오디오 노멀라이즈 (-14 LUFS)
6. Export
```

### 편집 시간 예상
| 원본 길이 | 편집 시간 | 최종 길이 |
|----------|----------|----------|
| 6-8분 | 10분 | 4-6분 |
| 10-12분 | 15분 | 7-9분 |
| 15-20분 | 25분 | 10-15분 |

---

## 2. Export 설정 (YouTube 최적)

```
포맷: MP4 (H.264)
해상도: 1920x1080
프레임레이트: 30fps
비트레이트: 16 Mbps (VBR)
오디오: AAC, 320kbps, 48kHz
```

### 파일명 규칙
```
[WIA Rust Learn] EP01 Why Rust.mp4
[WIA Rust Learn] EP02 Installing Rust.mp4
...
[WIA Rust Intermediate] EP01 Ownership Edge Cases.mp4
...
[WIA Rust Advanced] EP01 Memory Model.mp4
```

---

## 3. YouTube 업로드 템플릿

### 제목 형식
```
[WIA Rust {과정}] EP{번호}: {한글제목} | {영문제목}

예시:
[WIA Rust Learn] EP01: 왜 Rust인가? | Why Rust?
[WIA Rust Learn] EP08: 소유권 규칙 | Ownership Rules
[WIA Rust Intermediate] EP01: 소유권 엣지 케이스 | Ownership Edge Cases
[WIA Rust Advanced] EP01: 메모리 모델 | Memory Model
```

### 설명 템플릿 (복붙용)

```
WIA Rust {과정} - EP{번호}: {제목}

{한 줄 요약}

📚 이 시리즈 전체 보기:
▶️ Rust 입문 (28편): [플레이리스트 URL]
▶️ Rust 실무 (28편): [플레이리스트 URL]
▶️ Rust 고급 (28편): [플레이리스트 URL]

🔗 관련 링크:
• 슬라이드 (HTML): https://github.com/WIA-Official/wia-standards
• Rust 공식: https://www.rust-lang.org
• Rust Playground: https://play.rust-lang.org

⏰ 타임스탬프:
0:00 인트로
0:05 {첫 번째 주제}
1:30 {두 번째 주제}
...

📌 WIA (World Certification Industry Association)
弘益人間 - 널리 인간을 이롭게 하라

#WIA #Rust #러스트 #프로그래밍 #코딩 #개발자 #시스템프로그래밍
```

---

## 4. 태그 (복붙용)

### 공통 태그
```
WIA, Rust, 러스트, 프로그래밍, 코딩, 개발, 시스템프로그래밍, 메모리안전, 소유권, ownership, 프로그래밍언어, 개발자, 코딩강의, 무료강의
```

### Learn 추가 태그
```
러스트입문, 러스트기초, Rust기초, Rust입문, 러스트배우기, 프로그래밍입문
```

### Intermediate 추가 태그
```
러스트실무, 러스트중급, Rust중급, async, trait, 제네릭
```

### Advanced 추가 태그
```
러스트고급, 시스템프로그래밍, unsafe, FFI, 매크로, 임베디드
```

---

## 5. 썸네일 가이드

### 규격
```
크기: 1280 x 720 px
포맷: PNG 또는 JPG
용량: 2MB 이하
```

### 디자인 요소
```
┌─────────────────────────────┐
│  [WIA 로고]     EP{번호}   │
│                             │
│    🦀 {핵심 키워드}         │
│                             │
│  ─────────────────────────  │
│  RUST {과정}               │
└─────────────────────────────┘

배경색: #1a1a2e (다크) 또는 #f97316 (Rust 오렌지)
텍스트: 흰색 또는 검정, 굵은 폰트
```

---

## 6. 플레이리스트 구성

### 플레이리스트 제목
```
🦀 WIA Rust Learn - 입문 (28편)
🦀 WIA Rust Intermediate - 실무 (28편)
🦀 WIA Rust Advanced - 고급 (28편)
```

### 플레이리스트 설명
```
WIA Rust {과정} 과정입니다.

총 28편으로 구성되어 있으며, 순서대로 시청하시면 됩니다.

📚 전체 커리큘럼:
• Learn (입문): 기초 문법, 소유권, 기본 프로젝트
• Intermediate (실무): 비동기, 성능 최적화, 실무 패턴
• Advanced (고급): unsafe, FFI, 매크로, 시스템 프로그래밍

🔗 GitHub: https://github.com/WIA-Official/wia-standards

弘益人間 - Benefit All Humanity
© 2025 SmileStory Inc. / WIA
```

---

## 7. 업로드 후 체크리스트

### 즉시 (업로드 직후)
- [ ] 제목/설명 확인
- [ ] 썸네일 업로드
- [ ] 플레이리스트에 추가
- [ ] 공개 설정 (공개/예약)

### 처리 완료 후 (24시간 내)
- [ ] 자동 자막 검토 및 수정
- [ ] 엔드스크린 추가 (구독 + 다음 영상)
- [ ] 카드 추가 (관련 영상 링크)
- [ ] 첫 댓글 달기 (타임스탬프)

### 첫 댓글 템플릿
```
📌 타임스탬프
0:00 인트로
0:05 {주제1}
1:30 {주제2}
3:00 {주제3}
...

질문은 댓글로 남겨주세요! 🦀
```

---

## 8. 업로드 일정 (권장)

### 일일 업로드 시
```
월-금: 매일 1편
총 17주 (84편 ÷ 5편/주)
```

### 주 3회 업로드 시
```
월/수/금: 각 1편
총 28주 (84편 ÷ 3편/주)
```

### 예약 시간
```
한국 시간: 오후 6:00 (퇴근 시간)
또는: 오전 7:00 (출근 전)
```

---

## 9. 파일 관리

### 업로드 완료 후
```
~/Videos/WIA-Rust/
├── Learn/
│   ├── uploaded/     ← 업로드 완료
│   └── pending/      ← 대기 중
├── Intermediate/
└── Advanced/
```

### 백업
```
Google Drive 또는 외장하드에 원본 보관
최소 1년 보관 권장
```

---

**연계 문서:**
- `NARRATION_SCRIPTS.md` - 나레이션 스크립트
- `OBS_WORKFLOW.md` - 촬영 가이드

© 2025 SmileStory Inc. / WIA
