# Phase 3: Communication Protocol Standard
## Claude Code 작업 프롬프트

---

**Standard**: WIA Smart Home
**Phase**: 3 of 4
**Language**: **Rust** (Primary)
**목표**: Matter Protocol 기반 디바이스 통신 프로토콜 구현

---

## 🎯 목표

Matter Protocol과 호환되는 스마트홈 디바이스 통신 레이어 구현

---

## 📦 핵심 기능

```
1. Matter Protocol 어댑터
2. 디바이스 디스커버리
3. 메시지 직렬화/역직렬화
4. 보안 통신 (TLS/DTLS)
5. 접근성 확장 메시지
```

---

## 🔧 구현 범위

### 1. Protocol Adapter
- Matter 클러스터 매핑
- 커스텀 접근성 클러스터
- On/Off, Level Control, Color Control

### 2. Device Discovery
- mDNS/DNS-SD 기반 디스커버리
- 디바이스 등록/해제
- 연결 상태 모니터링

### 3. Message Format
- 표준 Matter 메시지 포맷
- WIA 접근성 확장 메시지
- JSON/Binary 직렬화

### 4. Security
- PAKE (Password Authenticated Key Exchange)
- TLS 1.3 / DTLS 1.3
- 인증서 관리

---

## 📁 프로젝트 구조

```
/api/rust/src/
├── protocol/
│   ├── mod.rs
│   ├── matter/
│   │   ├── mod.rs
│   │   ├── clusters.rs      # Matter 클러스터 정의
│   │   ├── messages.rs      # 메시지 포맷
│   │   └── adapter.rs       # Matter 어댑터
│   ├── discovery/
│   │   ├── mod.rs
│   │   └── mdns.rs          # mDNS 디스커버리
│   └── accessibility/
│       ├── mod.rs
│       └── extensions.rs    # WIA 접근성 확장
```

---

## 📋 참고 자료

```
- Matter Protocol Specification (CSA-IoT)
- RFC 6762 (mDNS)
- RFC 6763 (DNS-SD)
- WIA Smart Home Phase 1-2 스키마
```

---

## ✅ 완료 체크리스트

```
□ Protocol 모듈 구조 생성
□ Matter 클러스터 정의
□ 메시지 포맷 구현
□ 디바이스 디스커버리 구현
□ 접근성 확장 메시지 구현
□ 테스트 작성
□ README 업데이트
```

---

弘益人間 🤟🦀
