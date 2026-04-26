# WIA-RUST-LEARN Master Prompt

> **Zero-to-Rust**: 중간 언어 없이, 처음부터 Rust로!
> 弘益人間 - 널리 인간을 이롭게 하라

---

## 1. 철학: Zero-to-Rust

### 1.1 왜 "중간 언어 NO"인가?

```
기존 방식:                    Zero-to-Rust:
┌─────────────┐              ┌─────────────┐
│  Python     │              │             │
│     ↓       │              │    Rust     │
│  JavaScript │      VS      │   (처음부터) │
│     ↓       │              │             │
│   Rust      │              │             │
└─────────────┘              └─────────────┘
   3-6개월                      직접 시작!
```

**핵심 원칙:**
1. **두려움 제거**: Rust는 어렵지 않다, 다를 뿐이다
2. **개념 먼저**: 소유권을 코드 전에 이해
3. **시각화**: 메모리를 눈으로 보여준다
4. **실전 즉시**: 첫 날부터 실행 가능한 코드

### 1.2 학습 철학

```
"컴파일러가 친구다"

Rust 컴파일러는 까다로운 선생님이 아니다.
실수를 막아주는 수호자다.

에러 메시지 = 학습 기회
```

---

## 2. 웹 Playground 스펙

### 2.1 기술 스택

```yaml
Editor:
  engine: Monaco Editor
  theme: Dark (VS Code style)
  features:
    - Rust syntax highlighting
    - Auto-completion (rust-analyzer via WASM)
    - Inline error hints
    - Format on save (rustfmt)

Compiler:
  backend: Rust → WASM (wasm-pack)
  execution: Browser sandbox
  output: Console + Visual memory view

UI Framework:
  style: WIA Dark Theme
  colors:
    background: "#0f172a"
    primary: "#f97316"  # Rust Orange
    secondary: "#0EA5E9"
    text: "#f8fafc"
```

### 2.2 Playground 레이아웃

```
┌─────────────────────────────────────────────────────────────┐
│  WIA Rust Playground                    [Run] [Share] [?]   │
├────────────────────────────┬────────────────────────────────┤
│                            │  📊 Memory Visualizer          │
│   // Your Rust code        │  ┌─────────────────────────┐   │
│   fn main() {              │  │ Stack    │ Heap         │   │
│       let x = 5;           │  │ ┌─────┐  │ ┌─────────┐  │   │
│       let y = &x;          │  │ │ x:5 │  │ │         │  │   │
│   }                        │  │ │ y:→ │──┼─│         │  │   │
│                            │  │ └─────┘  │ └─────────┘  │   │
│                            │  └─────────────────────────┘   │
├────────────────────────────┴────────────────────────────────┤
│  Console Output                                              │
│  > Compiling...                                              │
│  > Success! x = 5                                            │
└─────────────────────────────────────────────────────────────┘
```

### 2.3 소유권 시각화 애니메이션

```typescript
interface OwnershipAnimation {
  // 변수 생성
  create: {
    variable: string;
    value: any;
    location: "stack" | "heap";
    animation: "fade-in" | "slide-down";
  };

  // 이동 (Move)
  move: {
    from: string;
    to: string;
    animation: "slide-arrow";
    result: "original-grayed-out";
  };

  // 빌림 (Borrow)
  borrow: {
    owner: string;
    borrower: string;
    type: "immutable" | "mutable";
    animation: "dotted-line";
    color: "green" | "yellow";
  };

  // 해제 (Drop)
  drop: {
    variable: string;
    animation: "fade-out-with-particles";
    scope_end: boolean;
  };
}
```

**시각화 컬러 코드:**
| 상태 | 색상 | 의미 |
|------|------|------|
| 소유 | 🟢 Green | 유효한 소유권 |
| 이동됨 | ⚫ Gray | 더 이상 사용 불가 |
| 불변 빌림 | 🔵 Blue | 읽기 전용 참조 |
| 가변 빌림 | 🟡 Yellow | 쓰기 가능 참조 |
| 해제 | 🔴 Red→사라짐 | 메모리 해제 |

---

## 3. 6레벨 커리큘럼

### Level 0: 준비 (2시간)
```yaml
목표: 환경 설정 & 첫 프로그램
내용:
  - Rust 설치 (rustup)
  - VS Code + rust-analyzer
  - Hello, World!
  - cargo 기초 (new, build, run)
실습: "Hello, WIA!" 출력
```

### Level 1: 기초 문법 (8시간)
```yaml
목표: Rust 기본 문법 마스터
내용:
  - 변수와 불변성
  - 데이터 타입 (scalar, compound)
  - 함수
  - 제어 흐름 (if, loop, while, for)
  - 주석과 문서화
실습: 간단한 계산기 CLI
```

### Level 2: 소유권 (12시간) ⭐ 핵심
```yaml
목표: Rust의 핵심 개념 완전 정복
내용:
  - 스택 vs 힙
  - 소유권 규칙 3가지
  - 이동(Move) 이해하기
  - 참조와 빌림(&, &mut)
  - 슬라이스
  - 라이프타임 기초
시각화: 모든 예제에 메모리 다이어그램
실습: 문자열 처리 유틸리티
```

### Level 3: 구조화 (10시간)
```yaml
목표: 데이터 구조와 모듈화
내용:
  - 구조체 (struct)
  - 열거형 (enum)
  - 패턴 매칭 (match)
  - 모듈 시스템
  - Crate와 패키지
실습: 연락처 관리 앱
```

### Level 4: 고급 기능 (12시간)
```yaml
목표: Rust의 강력한 기능들
내용:
  - 제네릭
  - 트레이트
  - 라이프타임 심화
  - 클로저
  - 이터레이터
  - 스마트 포인터 (Box, Rc, RefCell)
실습: 미니 데이터베이스 엔진
```

### Level 5: 동시성 (10시간)
```yaml
목표: 안전한 병렬 프로그래밍
내용:
  - 스레드
  - 메시지 전달 (channel)
  - 공유 상태 (Mutex, Arc)
  - Send와 Sync 트레이트
  - async/await 기초
실습: 멀티스레드 웹 크롤러
```

### Level 6: 실전 프로젝트 (20시간)
```yaml
목표: 실제 프로젝트 완성
프로젝트 선택:
  1. CLI 도구 (clap + 파일 처리)
  2. 웹 서버 (Actix-web/Axum)
  3. WASM 앱 (Yew/Leptos)
  4. 시스템 유틸리티 (OS 연동)
```

---

## 4. 4대 실전 프로젝트

### 프로젝트 1: WIA-CLI Tool
```rust
// WIA 표준 검증 CLI 도구
use clap::Parser;

#[derive(Parser)]
#[command(name = "wia-validate")]
struct Cli {
    #[arg(short, long)]
    standard: String,  // e.g., "WIA-HOME"

    #[arg(short, long)]
    file: PathBuf,
}

fn main() -> Result<()> {
    let cli = Cli::parse();
    let validator = WiaValidator::new(&cli.standard)?;
    validator.check(&cli.file)?;
    println!("✅ Valid WIA-{} format!", cli.standard);
    Ok(())
}
```

### 프로젝트 2: Rust-Core Simulator
```rust
// WIA AI-City의 Rust-Core 시뮬레이터
pub struct RustCore {
    hbm_manager: HbmManager,
    power_controller: PowerController,
    thermal_monitor: ThermalMonitor,
}

impl RustCore {
    pub fn allocate_hbm(&mut self, size: usize) -> Result<HbmBlock> {
        // 소유권 시연: HbmBlock은 이동됨
        self.hbm_manager.allocate(size)
    }

    pub fn get_status(&self) -> &SystemStatus {
        // 불변 빌림 시연
        &self.status
    }
}
```

### 프로젝트 3: WIA Protocol Parser
```rust
// AIDC-Link 프로토콜 파서
use nom::{bytes::complete::*, IResult};

pub fn parse_aidc_message(input: &[u8]) -> IResult<&[u8], AidcMessage> {
    let (input, header) = parse_header(input)?;
    let (input, payload) = parse_payload(input, header.length)?;
    let (input, checksum) = parse_checksum(input)?;

    Ok((input, AidcMessage { header, payload, checksum }))
}

// 라이프타임 실습: 파서가 입력 슬라이스를 빌림
```

### 프로젝트 4: Sharing Engine
```rust
// 1% 나눔 엔진 구현
use rust_decimal::Decimal;

pub struct SharingEngine {
    education_fund: Decimal,   // 0.3%
    healthcare_fund: Decimal,  // 0.3%
    environment_fund: Decimal, // 0.2%
    emergency_fund: Decimal,   // 0.2%
}

impl SharingEngine {
    pub fn process_transaction(&mut self, amount: Decimal) -> ShareResult {
        let contribution = amount * Decimal::from_str("0.01")?;

        // Arc<Mutex<>> 활용한 동시성 안전 분배
        self.distribute(contribution)
    }
}
```

---

## 5. 유튜브 시리즈 구성 (19편)

### 시즌 1: 시작하기 (1-4편)
```yaml
EP01: "Rust, 왜 배워야 할까?" (15분)
  - Rust의 장점
  - 취업 시장
  - Zero-to-Rust 철학 소개

EP02: "5분 만에 Rust 설치하기" (8분)
  - rustup 설치
  - VS Code 설정
  - 첫 cargo 프로젝트

EP03: "Hello, Rust! 첫 프로그램" (12분)
  - main 함수
  - println! 매크로
  - 변수와 타입

EP04: "Rust의 불변성, 왜 중요할까?" (15분)
  - let vs let mut
  - 불변성의 장점
  - 섀도잉
```

### 시즌 2: 소유권 마스터 (5-10편)
```yaml
EP05: "스택 vs 힙, 눈으로 보기" (18분)
  - 메모리 레이아웃
  - 시각화 데모
  - 성능 차이

EP06: "소유권 3가지 규칙" (20분)
  - 규칙 1: 하나의 소유자
  - 규칙 2: 스코프 끝에서 해제
  - 규칙 3: 이동(Move)

EP07: "이동(Move)을 눈으로 보자" (15분)
  - 애니메이션 시연
  - Copy 트레이트
  - Clone vs Copy

EP08: "참조와 빌림 완전정복" (20분)
  - &와 &mut
  - 빌림 규칙
  - 댕글링 참조 방지

EP09: "라이프타임 기초" (18분)
  - 'a 표기법
  - 함수에서 라이프타임
  - 컴파일러 추론

EP10: "소유권 실전 연습" (25분)
  - 10가지 연습 문제
  - 라이브 코딩
  - 흔한 실수들
```

### 시즌 3: 구조와 패턴 (11-14편)
```yaml
EP11: "구조체로 데이터 묶기" (15분)
EP12: "열거형과 Option" (18분)
EP13: "match로 패턴 매칭" (20분)
EP14: "모듈로 코드 정리" (15분)
```

### 시즌 4: 고급 & 동시성 (15-18편)
```yaml
EP15: "제네릭으로 코드 재사용" (18분)
EP16: "트레이트 = Rust의 인터페이스" (20분)
EP17: "스레드와 메시지 전달" (22분)
EP18: "공유 상태와 Mutex" (20분)
```

### 시즌 5: 실전 (19편)
```yaml
EP19: "WIA 표준 CLI 도구 만들기" (45분)
  - 전체 프로젝트 라이브 코딩
  - 지금까지 배운 모든 것 활용
```

---

## 6. WIA 인증 시스템

### 인증 레벨

```
┌─────────────────────────────────────────────────────────────┐
│  🏆 WIA Rust Certification Levels                           │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  💎 Diamond    완전체 - 모든 영역 마스터                    │
│       ↑       • 오픈소스 기여                               │
│  🥇 Platinum   실전 전문가                                  │
│       ↑       • Level 6 프로젝트 완료                       │
│  🥈 Gold       동시성 전문가                                 │
│       ↑       • Level 5 완료 + 멀티스레드 프로젝트          │
│  🥉 Silver     Rust 개발자                                   │
│       ↑       • Level 3-4 완료                              │
│  🟤 Bronze     입문자                                        │
│               • Level 0-2 완료 (소유권 이해)                │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

### 인증 시험 구조

```yaml
Bronze Exam:
  duration: 60분
  format: 온라인
  sections:
    - 기초 문법 (20문항)
    - 소유권 개념 (15문항)
    - 코드 읽기 (5문항)
  passing: 70%

Silver Exam:
  duration: 90분
  format: 온라인 + 실습
  sections:
    - 개념 문제 (20문항)
    - 코드 작성 (3문제)
    - 디버깅 (2문제)
  passing: 75%

Gold Exam:
  duration: 120분
  format: 실기 중심
  sections:
    - 미니 프로젝트 구현
    - 코드 리뷰
    - 성능 최적화
  passing: 80%

Platinum Exam:
  duration: 3시간
  format: 프로젝트 기반
  task: 실전 애플리케이션 개발
  review: 전문가 코드 리뷰

Diamond:
  requirements:
    - Platinum 취득
    - 오픈소스 PR 5개 이상 머지
    - 또는 Rust 교육 자료 기여
```

---

## 7. Ebook 8챕터 구성

### English Edition

```yaml
Chapter 1: "Why Rust?"
  - The Memory Safety Revolution
  - Performance Without Garbage Collection
  - Zero-Cost Abstractions

Chapter 2: "Getting Started"
  - Installation & Setup
  - Your First Program
  - Understanding Cargo

Chapter 3: "Ownership: The Heart of Rust"
  - Stack vs Heap
  - The Three Rules
  - Move, Copy, Clone

Chapter 4: "Borrowing & Lifetimes"
  - References Explained
  - Mutable Borrowing
  - Lifetime Annotations

Chapter 5: "Structuring Your Code"
  - Structs and Enums
  - Pattern Matching
  - Modules and Crates

Chapter 6: "Advanced Features"
  - Generics
  - Traits
  - Smart Pointers

Chapter 7: "Fearless Concurrency"
  - Threads in Rust
  - Message Passing
  - Shared State

Chapter 8: "Building Real Projects"
  - CLI Tools
  - Web Services
  - WIA Standard Integration
```

### Korean Edition

```yaml
1장: "왜 Rust인가?"
  - 메모리 안전성 혁명
  - GC 없는 성능
  - 제로코스트 추상화

2장: "시작하기"
  - 설치와 설정
  - 첫 프로그램
  - Cargo 이해하기

3장: "소유권: Rust의 심장"
  - 스택 vs 힙
  - 세 가지 규칙
  - 이동, 복사, 클론

4장: "빌림과 라이프타임"
  - 참조 설명
  - 가변 빌림
  - 라이프타임 표기

5장: "코드 구조화"
  - 구조체와 열거형
  - 패턴 매칭
  - 모듈과 크레이트

6장: "고급 기능"
  - 제네릭
  - 트레이트
  - 스마트 포인터

7장: "두려움 없는 동시성"
  - Rust의 스레드
  - 메시지 전달
  - 공유 상태

8장: "실전 프로젝트"
  - CLI 도구
  - 웹 서비스
  - WIA 표준 연동
```

---

## 8. 디자인 시스템

### 컬러 팔레트

```css
:root {
  /* Rust 테마 */
  --rust-orange: #f97316;
  --rust-dark: #c2410c;

  /* WIA Dark Theme */
  --bg-primary: #0f172a;
  --bg-secondary: #1e293b;
  --bg-tertiary: #334155;

  /* Text */
  --text-primary: #f8fafc;
  --text-secondary: #94a3b8;

  /* Accent */
  --accent-blue: #0EA5E9;
  --accent-green: #10b981;
  --accent-yellow: #eab308;
  --accent-red: #ef4444;
}
```

### 코드 블록 스타일

```css
.code-block {
  background: #1e1e1e;
  border-radius: 8px;
  padding: 16px;
  font-family: 'JetBrains Mono', 'Fira Code', monospace;
  font-size: 14px;
  line-height: 1.6;
  overflow-x: auto;
  border-left: 4px solid var(--rust-orange);
}

/* Rust 문법 하이라이팅 */
.rust-keyword { color: #569cd6; }
.rust-type { color: #4ec9b0; }
.rust-function { color: #dcdcaa; }
.rust-string { color: #ce9178; }
.rust-comment { color: #6a9955; }
.rust-macro { color: #c586c0; }
.rust-lifetime { color: #d7ba7d; }
```

---

## 9. 웹사이트 구조

```
wia-rust-learn/
├── index.html              # 랜딩 페이지
├── playground/
│   └── index.html          # 웹 Playground
├── curriculum/
│   ├── level-0.html
│   ├── level-1.html
│   ├── level-2.html
│   ├── level-3.html
│   ├── level-4.html
│   ├── level-5.html
│   └── level-6.html
├── ebook/
│   ├── en/
│   │   ├── index.html
│   │   ├── chapter-01.html
│   │   └── ...
│   └── ko/
│       ├── index.html
│       ├── chapter-01.html
│       └── ...
├── certification/
│   └── index.html
└── spec/
    ├── CURRICULUM.md
    ├── PLAYGROUND.md
    └── CERTIFICATION.md
```

---

## 10. API 스펙

### Playground API

```typescript
// Compile & Run
POST /api/compile
{
  "code": string,
  "edition": "2021" | "2018" | "2015",
  "mode": "debug" | "release",
  "visualize": boolean
}

Response:
{
  "success": boolean,
  "output": string,
  "errors": CompilerError[],
  "visualization"?: MemoryVisualization
}

// Share Code
POST /api/share
{
  "code": string,
  "title"?: string
}

Response:
{
  "shareId": string,
  "url": string
}
```

### Certification API

```typescript
// Start Exam
POST /api/certification/start
{
  "level": "bronze" | "silver" | "gold" | "platinum",
  "userId": string
}

// Submit Answer
POST /api/certification/submit
{
  "examId": string,
  "questionId": string,
  "answer": string | CodeSubmission
}

// Get Result
GET /api/certification/result/{examId}
```

---

## 11. 통합: WIA 생태계

### WIA 표준 연동

```rust
// WIA-RUST-LEARN은 다른 WIA 표준과 연동
use wia_home::SmartDevice;
use wia_social::Profile;
use wia_ai_city::RustCore;

fn learn_with_real_examples() {
    // Level 4 프로젝트: WIA-HOME 디바이스 제어
    let device = SmartDevice::new("light-001");

    // Level 5 프로젝트: WIA-AI-CITY Rust-Core
    let mut core = RustCore::new();
    core.allocate_hbm(1024)?;

    // Level 6 프로젝트: 풀스택 WIA 앱
    let app = WiaApp::builder()
        .with_home(device)
        .with_ai_core(core)
        .build()?;
}
```

---

## 12. 메타데이터

```yaml
Standard: WIA-RUST-LEARN
Version: 1.0.0
Status: Draft
Created: 2025-01-01
Authors: WIA Standards Committee

ISBN:
  English: 979-8-XXXX-XXXX-X
  Korean: 979-11-XXXX-XXXX-X

Price: $79 USD / ₩79,000 KRW

License: MIT

Philosophy: 弘益人間 - Benefit All Humanity
```

---

© 2025 WIA - World Certification Industry Association
弘益人間 (홍익인간) · Benefit All Humanity
