# Phase 3: Communication Protocol Standard
## Claude Code 작업 프롬프트

---

**Standard**: WIA Education
**Phase**: 3 of 4
**Language**: **Rust** (Primary)
**목표**: LTI/xAPI 기반 교육 접근성 통신 프로토콜 구현

---

## 🎯 목표

Learning Tools Interoperability (LTI) 및 xAPI와 호환되는 교육 접근성 데이터 교환 프로토콜 구현

---

## 📦 핵심 기능

```
1. LTI 1.3 Protocol Adapter
2. xAPI Statement Generator
3. Profile Sync Protocol
4. Accessibility Request/Response Protocol
5. Real-time Adaptation Events
```

---

## 🔧 구현 범위

### 1. LTI 1.3 Adapter
- LTI 메시지 타입 (ResourceLink, DeepLinking)
- 접근성 프로필 전송 (Custom Claims)
- OAuth 2.0 인증 흐름
- Platform/Tool 통신

### 2. xAPI Integration
- 학습 활동 Statement 생성
- 접근성 적용 이벤트 기록
- 학습자 프로필 Activity
- Verb/Object 정의

### 3. Profile Sync Protocol
- JSON-LD 기반 프로필 포맷
- 버전 관리 및 충돌 해결
- 증분 동기화
- 암호화 전송

### 4. Accessibility Events
- 실시간 적응 요청
- WebSocket 기반 이벤트 스트림
- 접근성 설정 변경 알림
- 콘텐츠 대안 요청

### 5. Message Security
- JWT 기반 인증
- TLS 1.3 전송 보안
- 개인정보 보호 (FERPA/GDPR 준수)

---

## 📁 프로젝트 구조

```
/api/rust/src/
├── protocol/
│   ├── mod.rs
│   ├── lti/
│   │   ├── mod.rs
│   │   ├── messages.rs       # LTI 메시지 타입
│   │   ├── claims.rs         # JWT Claims (접근성 포함)
│   │   └── adapter.rs        # LTI Platform/Tool 어댑터
│   ├── xapi/
│   │   ├── mod.rs
│   │   ├── statements.rs     # xAPI Statement 구조
│   │   ├── verbs.rs          # 접근성 관련 Verb
│   │   └── generator.rs      # Statement 생성기
│   ├── sync/
│   │   ├── mod.rs
│   │   ├── profile_sync.rs   # 프로필 동기화
│   │   └── conflict.rs       # 충돌 해결
│   └── events/
│       ├── mod.rs
│       ├── accessibility.rs  # 접근성 이벤트
│       └── stream.rs         # 이벤트 스트림
```

---

## 📋 참고 자료

```
- IMS LTI 1.3 Specification
- xAPI (Experience API) Specification
- AccessForAll 3.0 (ISO 24751)
- JSON-LD 1.1 Specification
- WIA Education Phase 1-2 스키마
```

---

## ✅ 완료 체크리스트

```
□ Protocol 모듈 구조 생성
□ LTI 1.3 메시지/어댑터 구현
□ xAPI Statement 생성기 구현
□ Profile Sync 프로토콜 구현
□ 접근성 이벤트 스트림 구현
□ 테스트 작성
□ README 업데이트
```

---

弘益人間 🤟🦀
