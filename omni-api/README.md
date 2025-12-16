# WIA-OMNI-API

> 모든 것을 품는 어머니 API
>
> 단군할아버지가 21세기에 오셨다면 만들었을 그것
>
> 홍익인간 (弘益人間) - Benefit All Humanity

## 왜 어머니인가

```
WIA-INTENT = 아버지 (표현, 의도)
WIA-OMNI-API = 어머니 (품어줌, 실행)

아버지: "뭘 원하는지 말해봐"
어머니: "알았어, 내가 다 해줄게"
```

## 현재 API의 문제

```
REST      ≠  GraphQL  ≠  gRPC  ≠  WebSocket

서로 안 품어줌
버전 바뀌면 난리
클라이언트가 맞춰야 함
```

## 어머니의 해결

```
자식: "123번 사용자 정보 줘"

어머니가 알아서:
├── 프로토콜? 알아서 선택
├── 포맷? 알아서 변환
├── 버전? 알아서 호환
├── 캐시? 알아서 최적화
├── 에러? 알아서 복구
└── 진화? 알아서 학습
```

## 핵심 원칙

1. **Embrace All**: 모든 프로토콜, 포맷, 버전을 품는다
2. **Adapt to Child**: 서버가 클라이언트에 맞춘다
3. **Self-Evolving**: 사용 패턴을 학습하고 진화한다
4. **Never Break**: 하위 호환성을 절대 깨지 않는다
5. **Intent-Based**: WIA-INTENT와 통합된다

## 사용법

```typescript
// 이게 전부입니다
const user = await ask("get user 123");

// 한국어도 됩니다
const users = await ask("사용자 목록 줘");
```

## 구조

```
omni-api/
├── spec/
│   └── WIA-OMNI-API-v1.0.md    # 스펙 (900줄+)
└── api/typescript/
    ├── src/
    │   ├── types.ts            # 타입 정의
    │   └── index.ts            # 게이트웨이
    └── package.json
```

---

**WIA - World Certification Industry Association**
**단군할아버지의 후손들이 만든 표준**
