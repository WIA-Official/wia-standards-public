# WIA-RUST-ADVANCED 나레이션 스크립트

> YouTube @yeonsamheum 채널용 고급 과정
> 28편 전체 슬라이드별 나레이션

---

# Level 0: Unsafe 마스터

## EP 0-01: Memory Model

### 슬라이드 1
> "고급 과정 시작! Rust 메모리 모델을 제대로 이해해봐요."

### 슬라이드 2
> "Rust는 C++ 메모리 모델 기반이에요. happens-before 관계가 핵심!"
**마우스:** 다이어그램

### 슬라이드 3
> "Ordering 종류예요. Relaxed, Acquire, Release, AcqRel, SeqCst!"

### 슬라이드 4
> "Relaxed는 순서 보장 없어요. 카운터에만 쓰세요."

### 슬라이드 5
> "Acquire-Release 쌍이에요. 락 구현의 기본!"

### 슬라이드 6
> "SeqCst는 전역 순서 보장! 가장 안전하지만 비용 있어요."

### 슬라이드 7
> "data race vs race condition 차이! data race만 UB예요."

### 슬라이드 8
> "메모리 모델 이해하면 lock-free 코드 작성 가능!"

---

## EP 0-02: Undefined Behavior

### 슬라이드 1
> "UB 완벽 가이드! 절대 하면 안 되는 것들이에요."

### 슬라이드 2
> "Rust의 UB 목록이에요. C/C++보다 적지만 있어요!"
**마우스:** UB 리스트

### 슬라이드 3
> "댕글링 참조가 대표적 UB예요. use-after-free!"

### 슬라이드 4
> "data race도 UB예요! 동시 쓰기/읽기 금지!"

### 슬라이드 5
> "잘못된 값 생성도 UB예요. bool에 2 넣기!"

### 슬라이드 6
> "null 참조도 UB예요. &*ptr::null()!"

### 슬라이드 7
> "UB가 있으면 컴파일러가 뭐든 해도 돼요. 무서운 최적화!"

### 슬라이드 8
> "safe Rust면 UB 없어요! unsafe는 신중하게!"

---

## EP 0-03: Miri

### 슬라이드 1
> "Miri! unsafe 코드 검증 도구예요."

### 슬라이드 2
> "Miri는 Rust 인터프리터예요. UB를 런타임에 감지!"
**마우스:** 설치 명령어

### 슬라이드 3
> "cargo +nightly miri test로 실행해요!"

### 슬라이드 4
> "메모리 에러 감지예요. out-of-bounds, use-after-free!"

### 슬라이드 5
> "data race 감지예요! 멀티스레드 테스트!"

### 슬라이드 6
> "Stacked Borrows 검증이에요. 빌림 규칙 런타임 체크!"

### 슬라이드 7
> "CI에 Miri 넣으세요! unsafe 코드 품질 보장!"

### 슬라이드 8
> "Miri 통과해도 100% 안전은 아니에요. 하지만 큰 도움!"

---

# Level 1: FFI 완전정복

## EP 1-01: C Bindings

### 슬라이드 1
> "C 라이브러리 바인딩! Rust에서 C 호출하기!"

### 슬라이드 2
> "extern \"C\" 블록이에요. C 함수 선언!"
**마우스:** extern 블록

### 슬라이드 3
> "C 타입 매핑이에요. c_int, c_char, c_void!"

### 슬라이드 4
> "CString과 CStr이에요. Rust ↔ C 문자열!"

### 슬라이드 5
> "포인터 처리예요. *const, *mut!"

### 슬라이드 6
> "에러 처리예요. errno 체크!"

### 슬라이드 7
> "링커 설정이에요. build.rs에서 라이브러리 링크!"

### 슬라이드 8
> "안전한 래퍼 만들기! unsafe를 safe로 감싸요!"

---

## EP 1-02: Bindgen

### 슬라이드 1
> "bindgen과 cbindgen! 자동 바인딩 생성!"

### 슬라이드 2
> "bindgen은 C → Rust 바인딩 생성이에요!"
**마우스:** 명령어

### 슬라이드 3
> "cbindgen은 Rust → C 헤더 생성이에요!"

### 슬라이드 4
> "build.rs에서 자동 생성! 빌드할 때마다!"

### 슬라이드 5
> "allowlist로 필요한 것만! 바인딩 크기 줄이기!"

### 슬라이드 6
> "cbindgen.toml 설정이에요. 헤더 커스텀!"

### 슬라이드 7
> "#[repr(C)] 잊지 마세요! 메모리 레이아웃 호환!"

### 슬라이드 8
> "자동화로 유지보수 쉽게!"

---

## EP 1-03: PyO3 & napi-rs

### 슬라이드 1
> "Python과 Node.js에서 Rust 쓰기!"

### 슬라이드 2
> "PyO3로 Python 모듈 만들기! #[pyfunction]"
**마우스:** 어트리뷰트

### 슬라이드 3
> "maturin으로 빌드하고 배포해요!"

### 슬라이드 4
> "napi-rs로 Node.js 네이티브 모듈! #[napi]"

### 슬라이드 5
> "타입 변환이에요. Python/JS 타입 ↔ Rust 타입!"

### 슬라이드 6
> "에러 처리예요. PyResult, napi::Result!"

### 슬라이드 7
> "성능 비교! Python 대비 10-100배 빠를 수 있어요!"

### 슬라이드 8
> "핫 패스만 Rust로! 나머지는 그대로!"

---

# Level 2: 매크로 마스터

## EP 2-01: Proc Macro

### 슬라이드 1
> "절차적 매크로! derive 직접 만들기!"

### 슬라이드 2
> "proc-macro 크레이트 타입이에요. lib.rs에 proc-macro = true"
**마우스:** Cargo.toml

### 슬라이드 3
> "세 종류예요. derive, attribute, function-like!"

### 슬라이드 4
> "TokenStream이 입출력이에요! 토큰 조작!"

### 슬라이드 5
> "derive 매크로예요. #[proc_macro_derive(MyTrait)]"

### 슬라이드 6
> "attribute 매크로예요. #[proc_macro_attribute]"

### 슬라이드 7
> "function-like 매크로예요. #[proc_macro]"

### 슬라이드 8
> "다음에 syn과 quote로 쉽게 만들어요!"

---

## EP 2-02: syn & quote

### 슬라이드 1
> "syn과 quote! 매크로 작성의 필수 크레이트!"

### 슬라이드 2
> "syn은 파싱이에요. TokenStream → AST!"
**마우스:** parse_macro_input!

### 슬라이드 3
> "DeriveInput, ItemFn 같은 AST 타입이에요!"

### 슬라이드 4
> "quote는 코드 생성이에요. AST → TokenStream!"

### 슬라이드 5
> "quote! 매크로 안에서 #변수로 삽입해요!"

### 슬라이드 6
> "#(#iter),* 로 반복 생성해요!"

### 슬라이드 7
> "실전 예제: Builder derive 매크로!"

### 슬라이드 8
> "syn + quote = 강력한 코드 생성!"

---

## EP 2-03: IDE-Friendly Macros

### 슬라이드 1
> "IDE 친화적 매크로! rust-analyzer가 이해하게!"

### 슬라이드 2
> "Span 보존이 핵심! 에러 위치가 정확해야 해요!"
**마우스:** span 다이어그램

### 슬라이드 3
> "proc_macro2::Span::call_site() vs mixed_site()!"

### 슬라이드 4
> "quote_spanned! 로 스팬 지정해요!"

### 슬라이드 5
> "타입 힌트 제공! 숨겨진 타입도 명시!"

### 슬라이드 6
> "compile_error! 로 친절한 에러 메시지!"

### 슬라이드 7
> "문서화! #[doc = \"...\"] 자동 생성!"

### 슬라이드 8
> "좋은 매크로는 IDE에서도 잘 동작해요!"

---

# Level 3-9: (빠른 참조)

| EP | 제목 | 핵심 포인트 |
|----|------|------------|
| 3-01 | Clippy Lint | declare_clippy_lint!, LateLintPass |
| 3-02 | Compile-Time | const fn, static_assertions |
| 4-01 | no_std | #![no_std], core, alloc |
| 4-02 | Bare-Metal | volatile, MMIO |
| 4-03 | RTOS (Embassy) | async embedded, tasks |
| 5-01 | Bootloader | x86, VGA buffer |
| 5-02 | Mini Kernel | IDT, paging |
| 5-03 | Drivers | PCI, interrupts |
| 6-01 | GlobalAlloc | Layout, allocate |
| 6-02 | Memory Pools | arena, pool |
| 7-01 | SIMD | std::simd, portable |
| 7-02 | Inline Asm | asm!, in/out |
| 7-03 | CPU Optimize | cache, prefetch |
| 8-01 | wasm-pack | wasm32-unknown |
| 8-02 | wasm-bindgen | web-sys, js-sys |
| 9-01 | Interpreter | lexer, parser, eval |
| 9-02 | Game Engine | ECS, rendering |
| 9-03 | Distributed | raft, consensus |
| 9-04 | Mastery | 축하합니다! |

---

# 마스터 코스 완주 나레이션

## EP 9-04: Rust Master!

### 슬라이드 1
> "축하합니다! 드디어 고급 과정을 완주했어요!"

### 슬라이드 2
> "입문 28편, 중급 28편, 고급 28편! 총 84편을 마스터했어요!"

### 슬라이드 3
> "여러분이 배운 것들이에요. 소유권부터 OS 개발까지!"
**마우스:** 학습 내용 목록

### 슬라이드 4
> "이제 여러분은 무엇이든 만들 수 있어요!"

### 슬라이드 5
> "추천 다음 단계: 오픈소스 기여, 블로그 작성!"

### 슬라이드 6
> "커뮤니티에 참여하세요! Rust Korea, Discord!"

### 슬라이드 7
> "弘益人間 - 널리 인간을 이롭게 하라! 여러분의 Rust로 세상을 바꾸세요!"

### 슬라이드 8
> "감사합니다! 구독과 좋아요, 그리고 다른 분들께 공유해주세요!"
**마우스:** 하트 그리기, 손 흔들기

---

© 2025 WIA / @yeonsamheum
弘益人間 - 널리 인간을 이롭게 하라
