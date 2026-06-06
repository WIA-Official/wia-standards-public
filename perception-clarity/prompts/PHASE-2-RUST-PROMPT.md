# Phase 2: API Interface Standard (Rust)
## Claude Code 작업 프롬프트

---

**Standard**: WIA Perception Clarity  
**Phase**: 2 of 4  
**Language**: **Rust** (Primary)
**목표**: Rust 기반 고성능 PCI/보고 API 구현

---

## 🎯 목표

Rust로 표준 API 구현 — PCI 산출, 상태 매핑, `SensorClarityReport` 직렬화/검증. (TypeScript/Python은 바인딩으로 제공)

---

## 🦀 Rust 선택 이유

```
1. 성능: C++ 수준 속도 (실시간 센서 루프 적합)
2. 안전: 메모리 안전 보장
3. 현대적: 최신 언어 기능
4. 일관성: WIA Braille API와 동일 스택
```

---

## 📦 프로젝트 구조

```
/api/rust/
├── Cargo.toml
├── src/
│   ├── lib.rs           # 메인 라이브러리
│   ├── types.rs         # 타입 정의 (SensorClass, ClarityState, Contaminant)
│   ├── core/
│   │   ├── mod.rs
│   │   └── pci.rs       # PCI 산출 + 상태 매핑
│   ├── adapters/
│   │   ├── mod.rs
│   │   └── simulator.rs
│   └── error.rs         # 에러 타입
├── tests/
│   └── integration_test.rs
└── examples/
    └── basic_usage.rs
```

---

## 🔧 필수 의존성

```toml
[dependencies]
tokio = { version = "1", features = ["full"] }
serde = { version = "1", features = ["derive"] }
serde_json = "1"
thiserror = "1"
async-trait = "0.1"

# WebAssembly (브라우저용)
wasm-bindgen = { version = "0.2", optional = true }

# Python 바인딩
pyo3 = { version = "0.20", optional = true }

# Node.js 바인딩  
neon = { version = "0.10", optional = true }
```

---

## 🔄 작업 순서

```
1. Cargo.toml 생성
2. 타입 정의 (types.rs)
3. 코어 로직 구현 (core/pci.rs — 가중 합산 PCI, 상태 매핑)
4. 어댑터 구현 (adapters/)
5. 에러 처리 (error.rs)
6. 테스트 작성
7. 예제 코드 작성
8. (선택) WASM/Python/Node 바인딩
```

---

## 📋 참고 구현

```
WIA Braille API (Rust 참고):
https://github.com/WIA-Official/tactile-braille-reader
```

---

## 📁 산출물

```
/perception-clarity/api/rust/Cargo.toml
/perception-clarity/api/rust/src/*.rs
/perception-clarity/api/rust/tests/*.rs
/perception-clarity/api/rust/examples/*.rs
```

---

## ✅ 완료 체크리스트

```
□ Cargo.toml 생성
□ 타입 정의 완료
□ 코어 로직 구현 (PCI 산출/상태 매핑)
□ 어댑터 구현
□ 에러 처리 구현
□ 테스트 통과
□ 예제 코드 작성
□ README 업데이트
```

---

弘益人間 🤟🦀
