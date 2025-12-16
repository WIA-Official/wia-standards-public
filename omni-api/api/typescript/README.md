# WIA-OMNI-API

> 모든 것을 품는 어머니 API
>
> The Mother of All APIs
>
> 홍익인간 (弘益人間) - Benefit All Humanity

## 철학

```
어머니는 자식이 뭘 원하는지만 들으면
나머지는 다 알아서 해준다.

어머니는 자식에게 복잡함을 전가하지 않는다.
어머니는 자식이 변하라고 강요하지 않는다.
어머니는 모든 자식을 품는다.
```

## Installation

```bash
npm install @wia/omni-api
```

## Quick Start

```typescript
import { OmniGateway, ask } from '@wia/omni-api';

// 가장 간단한 사용법
const user = await ask("get user 123");

// 한국어도 됩니다
const users = await ask("사용자 목록 줘");

// 상세 요청
const gateway = new OmniGateway({ baseUrl: 'https://api.example.com' });
const response = await gateway.request({
  intent: "get user 123",
  hints: {
    priority: "speed"
  }
});
```

## Features

### 1. Intent-Based Requests

```typescript
// 이렇게만 말하면
await ask("get user 123");

// 어머니가 알아서 처리:
// - 프로토콜 선택 (REST? GraphQL? gRPC?)
// - 포맷 결정 (JSON? XML?)
// - 버전 호환성
// - 캐싱
// - 에러 복구
```

### 2. Never-Break Policy

```typescript
// 버전이 바뀌어도 기존 클라이언트는 절대 깨지지 않습니다
// v1 클라이언트가 v3 서버에 요청해도 동작합니다
```

### 3. Protocol Embracing

```typescript
// REST, GraphQL, gRPC, WebSocket 모두 지원
// 어머니가 최적의 프로토콜을 선택합니다
```

### 4. Self-Evolution

```typescript
// 사용 패턴을 학습하고 스스로 최적화합니다
const patterns = gateway.getDetectedPatterns();
const suggestions = gateway.getOptimizationSuggestions();
```

### 5. Compassionate Errors

```typescript
// 에러도 어머니처럼 - 혼내지 않고 도와줍니다
{
  "error": {
    "code": "USER_NOT_FOUND",
    "message": "User not found",
    "comfort": "걱정 마세요, 흔한 일이에요.",
    "recovery_options": [...]
  }
}
```

## WIA-INTENT Integration

```typescript
// WIA-INTENT 언어와 통합
await gateway.request({
  intent: {
    type: 'wia-intent',
    code: `
      intent GetUserDashboard {
        given: user_id
        want: dashboard_data
        constraints {
          freshness: <= 5.minutes
        }
      }
    `
  }
});
```

## API Reference

### `OmniGateway`

Main gateway class.

```typescript
const gateway = new OmniGateway({
  baseUrl: 'https://api.example.com',
  defaultProtocol: 'rest',
  defaultFormat: 'json',
  defaultTimeout: 30000,
  autoRetry: true,
  maxRetries: 3,
  cacheEnabled: true,
  evolutionEnabled: true,
});
```

### `request<T>(request: OmniRequest): Promise<OmniResponse<T>>`

Make a request to the API.

### `ask<T>(intent: string, payload?: any): Promise<T>`

Simple intent-based request.

### `configure(config: Partial<GatewayConfig>): void`

Configure the default gateway.

## Philosophy

```
단군할아버지의 홍익인간 (弘益人間)

- 널리 인간을 이롭게 하라
- 어머니처럼 모든 것을 품어라
- 자식이 편하면 그것이 답이다
- 강요하지 말고 도와줘라
- 절대 깨뜨리지 말라

이것이 WIA-OMNI-API의 정신입니다.
```

---

**WIA - World Certification Industry Association**
**홍익인간 (弘益人間) - Benefit All Humanity**
