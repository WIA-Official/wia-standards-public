# WIA-RUST-INTERMEDIATE 나레이션 스크립트

> YouTube @yeonsamheum 채널용 중급 과정
> 28편 전체 슬라이드별 나레이션

---

# Level 0: 리뷰 & 심화

## EP 0-01: Ownership Edge Cases

### 슬라이드 1
> "중급 과정에 오신 걸 환영해요! 오늘은 소유권의 엣지케이스를 다뤄볼게요."

### 슬라이드 2 (Partial Move)
> "구조체의 일부 필드만 이동시키는 Partial Move예요. 이동된 필드는 못 쓰지만 나머지는 쓸 수 있어요."
**마우스:** 이동되는 필드 가리키기

### 슬라이드 3 (Temporary Values)
> "임시값의 라이프타임이에요. let 바인딩 없이 쓰면 문장 끝에서 드롭돼요."

### 슬라이드 4
> "메서드 체이닝에서 임시값 주의! 참조 반환하면 댕글링 위험해요."

### 슬라이드 5
> "Self 타입과 소유권이에요. self, &self, &mut self 차이 확실히 알아야 해요."

### 슬라이드 6
> "Drop 순서예요. 필드는 선언 순서대로, 지역변수는 역순으로 드롭돼요."

### 슬라이드 7
> "ManuallyDrop으로 자동 드롭 방지! FFI나 union에서 써요."

### 슬라이드 8
> "소유권 엣지케이스 끝! 다음은 라이프타임 심화예요."

---

## EP 0-02: Lifetime Advanced

### 슬라이드 1
> "라이프타임 심화예요! HRTB와 생략 규칙을 배워요."

### 슬라이드 2 (생략 규칙)
> "세 가지 생략 규칙이에요. 입력 하나면 출력도 같은 라이프타임, &self 있으면 출력은 self 라이프타임."

### 슬라이드 3 (HRTB)
> "for<'a>가 HRTB예요. '어떤 라이프타임에 대해서도'라는 뜻이에요. 클로저 트레이트에서 많이 봐요."
**마우스:** for<'a> 부분 강조

### 슬라이드 4
> "라이프타임 서브타이핑이에요. 'a: 'b는 'a가 'b보다 오래 산다는 뜻!"

### 슬라이드 5
> "구조체의 복잡한 라이프타임이에요. 여러 참조 필드가 있을 때 각각 다른 라이프타임 가능해요."

### 슬라이드 6
> "impl 블록에서 라이프타임이에요. 메서드마다 다른 라이프타임 관계 가능해요."

### 슬라이드 7
> "'static의 의미예요. 프로그램 전체 동안 유효하거나, 소유 타입을 의미해요."

### 슬라이드 8
> "라이프타임 심화 끝! 컴파일러랑 싸우지 말고 친해지세요!"

---

## EP 0-03: Compiler Errors

### 슬라이드 1
> "컴파일러 에러 마스터! 에러 메시지 읽는 법을 배워요."

### 슬라이드 2
> "에러 구조예요. 에러 코드, 위치, 설명, 힌트 순서로 읽어요."

### 슬라이드 3 (E0382)
> "E0382 use of moved value! 가장 흔한 에러예요. 클론하거나 참조로 바꾸세요."
**마우스:** 에러 코드와 해결책

### 슬라이드 4 (E0502)
> "E0502 가변/불변 동시 빌림! 빌림 범위를 좁히거나 RefCell 쓰세요."

### 슬라이드 5 (E0106)
> "E0106 라이프타임 누락! 컴파일러가 추론 못 하면 명시해줘야 해요."

### 슬라이드 6
> "cargo clippy 활용이에요! 에러 전에 경고로 잡아줘요."

### 슬라이드 7
> "rust-analyzer 힌트 활용이에요! IDE가 실시간으로 알려줘요."

### 슬라이드 8
> "에러는 친구예요! 메모리 버그를 런타임 전에 잡아주니까요."

---

# Level 1: 고급 타입 시스템

## EP 1-01: Associated Types

### 슬라이드 1
> "Associated Types vs Generics! 언제 뭘 쓸지 알아봐요."

### 슬라이드 2
> "제네릭은 호출자가 타입 결정, Associated Type은 구현자가 결정해요."
**마우스:** 비교 다이어그램

### 슬라이드 3
> "Iterator의 Item이 대표적이에요. impl Iterator for MyIter { type Item = i32; }"

### 슬라이드 4
> "제네릭은 여러 구현 가능, Associated Type은 타입당 하나예요."

### 슬라이드 5
> "제네릭이 많으면 Associated Type 고려하세요. 시그니처가 깔끔해져요."

### 슬라이드 6
> "둘 다 쓸 수도 있어요! trait Graph<N, E> { type Node; }"

### 슬라이드 7
> "Associated Type에도 바운드 가능! type Item: Clone;"

### 슬라이드 8
> "정리: 호출자 선택 → 제네릭, 구현자 결정 → Associated Type!"

---

## EP 1-02: GAT (Generic Associated Types)

### 슬라이드 1
> "GAT! Rust 1.65에서 안정화된 기능이에요."

### 슬라이드 2
> "Associated Type에 제네릭 파라미터 붙이는 거예요. type Item<'a>;"
**마우스:** <'a> 부분

### 슬라이드 3
> "LendingIterator 예제예요. 빌린 값을 반환하는 이터레이터 만들 수 있어요."

### 슬라이드 4
> "async trait에서 GAT가 핵심이에요. Future의 라이프타임 표현!"

### 슬라이드 5
> "컬렉션 추상화에 좋아요. type Iter<'a> where Self: 'a;"

### 슬라이드 6
> "where 절 주의! Self: 'a 바운드 자주 필요해요."

### 슬라이드 7
> "실전 예제: 데이터베이스 커넥션 풀 추상화"

### 슬라이드 8
> "GAT는 고급 기능! 필요할 때 쓰세요."

---

## EP 1-03: PhantomData

### 슬라이드 1
> "PhantomData! 타입은 쓰지만 값은 안 쓸 때 필요해요."

### 슬라이드 2
> "unused type parameter 에러 해결책이에요."
**마우스:** 에러 메시지와 해결 코드

### 슬라이드 3
> "라이프타임 표시용이에요. PhantomData<&'a T>로 'a 사용 표시!"

### 슬라이드 4
> "variance 제어예요. PhantomData<fn(T)>는 contravariant!"

### 슬라이드 5
> "소유권 표시예요. PhantomData<T>는 T를 소유한다고 컴파일러에게 알려요."

### 슬라이드 6
> "FFI에서 많이 써요. C 포인터를 안전하게 래핑할 때!"

### 슬라이드 7
> "타입 레벨 프로그래밍이에요. 상태를 타입으로 표현!"

### 슬라이드 8
> "PhantomData는 제로 사이즈! 런타임 비용 없어요."

---

# Level 2: 고급 트레이트

## EP 2-01: Trait Objects

### 슬라이드 1
> "dyn Trait vs impl Trait! 언제 뭘 쓸지 알아봐요."

### 슬라이드 2
> "impl Trait는 정적 디스패치, dyn Trait는 동적 디스패치예요."
**마우스:** 비교 다이어그램

### 슬라이드 3
> "impl Trait는 컴파일 타임에 타입 결정, 더 빨라요."

### 슬라이드 4
> "dyn Trait는 런타임에 결정, 유연해요. Vec<Box<dyn Animal>> 가능!"

### 슬라이드 5
> "성능 차이예요. dyn은 vtable 조회 오버헤드가 있어요."

### 슬라이드 6
> "바이너리 크기! impl Trait는 모노모피제이션으로 커질 수 있어요."

### 슬라이드 7
> "선택 기준: 타입 수 적고 성능 중요하면 impl, 플러그인 시스템이면 dyn!"

### 슬라이드 8
> "둘 다 장단점 있어요! 상황에 맞게 선택하세요."

---

## EP 2-02: Object Safety

### 슬라이드 1
> "Object Safety! 왜 어떤 트레이트는 dyn으로 못 쓸까요?"

### 슬라이드 2
> "규칙 1: Self를 Sized로 쓰면 안 돼요."

### 슬라이드 3
> "규칙 2: 메서드가 제네릭이면 안 돼요."

### 슬라이드 4
> "규칙 3: Self를 반환하면 안 돼요. Clone이 object-safe 아닌 이유!"
**마우스:** Clone 트레이트 정의

### 슬라이드 5
> "where Self: Sized로 메서드 제외 가능해요."

### 슬라이드 6
> "우회법: Box<Self> 반환하거나 associated type 사용!"

### 슬라이드 7
> "실전: trait Animal + Clone 조합이 안 되는 이유"

### 슬라이드 8
> "Object Safety 이해하면 트레이트 설계가 쉬워져요!"

---

## EP 2-03: Blanket Implementations

### 슬라이드 1
> "Blanket Implementation! impl<T> Trait for T 패턴이에요."

### 슬라이드 2
> "표준 라이브러리 예제: impl<T: Display> ToString for T"
**마우스:** 코드 강조

### 슬라이드 3
> "조건부 구현이에요. 바운드 만족하면 자동으로 구현!"

### 슬라이드 4
> "Into/From 패턴이에요. From 구현하면 Into가 공짜!"

### 슬라이드 5
> "AsRef 패턴이에요. 제네릭 함수에서 유연한 입력 받기!"

### 슬라이드 6
> "주의: 충돌 가능해요! 구체 타입 구현이 우선!"

### 슬라이드 7
> "extension trait 패턴이에요. 외부 타입에 메서드 추가!"

### 슬라이드 8
> "Blanket impl로 보일러플레이트 제거하세요!"

---

# Level 3: 비동기 심화

## EP 3-01: Future Implementation

### 슬라이드 1
> "Future 직접 구현! async/await가 뭔지 정확히 알아봐요."

### 슬라이드 2
> "Future는 trait이에요. poll 메서드 하나!"
**마우스:** poll 시그니처

### 슬라이드 3
> "Poll::Pending은 아직, Poll::Ready(T)는 완료!"

### 슬라이드 4
> "Waker가 핵심! Pending 반환할 때 waker 저장해야 해요."

### 슬라이드 5
> "간단한 Future 직접 구현해봐요. Delay future!"

### 슬라이드 6
> "async fn이 이걸 자동 생성해요. 상태 머신으로!"

### 슬라이드 7
> "combinator들: map, and_then, join 직접 구현 가능!"

### 슬라이드 8
> "Future 이해하면 async 디버깅이 쉬워져요!"

---

## EP 3-02: Pin & Unpin

### 슬라이드 1
> "Pin과 Unpin! 왜 필요한지 알아봐요."

### 슬라이드 2
> "자기 참조 구조체 문제예요! 이동하면 참조가 깨져요."
**마우스:** 다이어그램

### 슬라이드 3
> "async fn이 자기 참조 구조체 생성해요! await 전후 변수 참조!"

### 슬라이드 4
> "Pin<P>는 이동 금지예요. 메모리 주소 고정!"

### 슬라이드 5
> "Unpin은 이동해도 괜찮다는 마커 trait이에요. 대부분 타입이 Unpin!"

### 슬라이드 6
> "!Unpin 타입만 Pin이 의미 있어요."

### 슬라이드 7
> "pin! 매크로로 스택에 고정, Box::pin으로 힙에 고정!"

### 슬라이드 8
> "Pin은 async의 안전성 보장! 이해하면 에러가 안 무서워요."

---

## EP 3-03: Runtime Internals

### 슬라이드 1
> "async 런타임 내부! Tokio가 어떻게 동작하는지 알아봐요."

### 슬라이드 2
> "런타임 구성요소: Executor, Reactor, Task Queue!"
**마우스:** 아키텍처 다이어그램

### 슬라이드 3
> "Executor는 Future를 poll해요. work-stealing 스케줄러!"

### 슬라이드 4
> "Reactor는 I/O 이벤트 감지해요. epoll, kqueue, IOCP!"

### 슬라이드 5
> "Waker가 Reactor와 Executor 연결해요!"

### 슬라이드 6
> "spawn으로 태스크 생성, JoinHandle로 결과 받아요."

### 슬라이드 7
> "current_thread vs multi_thread 런타임 차이!"

### 슬라이드 8
> "런타임 이해하면 성능 튜닝이 가능해요!"

---

# Level 4: 성능 최적화

## EP 4-01: Benchmarking

### 슬라이드 1
> "벤치마킹! criterion으로 정확하게 측정해요."

### 슬라이드 2
> "cargo bench는 불안정해요. criterion 쓰세요!"

### 슬라이드 3
> "criterion 설정이에요. Cargo.toml에 추가하고 benches 폴더!"
**마우스:** 설정 파일

### 슬라이드 4
> "기본 벤치마크 작성이에요. c.bench_function()!"

### 슬라이드 5
> "비교 벤치마크예요. 여러 구현 비교!"

### 슬라이드 6
> "파라미터화 벤치마크예요. 입력 크기별 성능!"

### 슬라이드 7
> "결과 분석이에요! HTML 리포트 생성됨!"

### 슬라이드 8
> "최적화 전에 항상 측정하세요!"

---

## EP 4-02: Profiling

### 슬라이드 1
> "프로파일링! 병목을 찾아요."

### 슬라이드 2
> "perf로 CPU 프로파일링해요. Linux에서!"

### 슬라이드 3
> "flamegraph로 시각화해요! cargo flamegraph"
**마우스:** 플레임그래프 이미지

### 슬라이드 4
> "힙 프로파일링이에요. heaptrack, valgrind!"

### 슬라이드 5
> "cargo-instruments로 macOS 프로파일링!"

### 슬라이드 6
> "최적화 팁: 핫 패스 찾아서 집중!"

### 슬라이드 7
> "컴파일러 최적화예요. release 모드, LTO!"

### 슬라이드 8
> "측정 → 분석 → 최적화 → 반복!"

---

# Level 5-9: (계속...)

[Level 5-9 나레이션은 동일한 패턴으로 계속...]

---

# 에피소드별 빠른 참조

| EP | 제목 | 핵심 키워드 |
|----|------|------------|
| 0-01 | Ownership Edge Cases | partial move, temporary |
| 0-02 | Lifetime Advanced | HRTB, for<'a> |
| 0-03 | Compiler Errors | E0382, E0502, E0106 |
| 1-01 | Associated Types | type Item = T |
| 1-02 | GAT | type Item<'a> |
| 1-03 | PhantomData | zero-size marker |
| 2-01 | Trait Objects | dyn vs impl |
| 2-02 | Object Safety | Sized, Self |
| 2-03 | Blanket Impl | impl<T> for T |
| 3-01 | Future Impl | poll, Waker |
| 3-02 | Pin & Unpin | self-referential |
| 3-03 | Runtime | executor, reactor |
| 4-01 | Benchmarking | criterion |
| 4-02 | Profiling | flamegraph |
| 5-01 | repr | repr(C), packed |
| 5-02 | Cache-Friendly | AoS vs SoA |
| 5-03 | Arenas | bumpalo |
| 6-01 | Custom Errors | thiserror |
| 6-02 | Error Chain | anyhow, context |
| 7-01 | Builder/Newtype | type safety |
| 7-02 | Typestate | compile-time state |
| 7-03 | Interior Mutability | Cell, RefCell |
| 8-01 | Property Testing | proptest |
| 8-02 | Fuzzing | cargo-fuzz |
| 9-01 | High-Perf Server | async, connection pool |
| 9-02 | Database | SQLx |
| 9-03 | gRPC | tonic |
| 9-04 | Graduation | 축하! |

---

© 2025 WIA / @yeonsamheum
弘益人間 - 널리 인간을 이롭게 하라
