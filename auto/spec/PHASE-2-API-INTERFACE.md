# WIA Autonomous Vehicle - Phase 2: API Interface

## Version
- **Version**: 1.0.0
- **Date**: 2025-12-16
- **Status**: Final

## Overview

본 문서는 WIA 자율주행 차량 접근성 표준의 REST API 인터페이스를 정의합니다.

## API Specification

완전한 API 사양은 OpenAPI 3.1.0 형식으로 제공됩니다:

**파일**: `openapi.yaml`

### 주요 엔드포인트

#### 1. 차량 관리
- `GET /vehicles` - 사용 가능한 차량 목록
- `GET /vehicles/{id}` - 차량 상세 정보
- `POST /vehicles/{id}/book` - 차량 예약

#### 2. 탑승 관리
- `POST /rides` - 탑승 요청
- `GET /rides/{id}` - 탑승 상태 조회
- `DELETE /rides/{id}` - 탑승 취소

#### 3. 접근성 프로필
- `GET /profiles/{userId}` - 사용자 접근성 프로필
- `PUT /profiles/{userId}` - 프로필 업데이트

#### 4. 실시간 통신
- WebSocket endpoint: `/ws/rides/{rideId}`
- 차량 위치, 상태, 알림 실시간 전송

## 인증

### OAuth 2.0 (PKCE)
모바일 앱을 위한 보안 인증:
```
Authorization Code Flow with PKCE
- 15분 토큰 만료
- Refresh token 지원
```

### API Keys
서버 간 통신을 위한 API 키:
```
Authorization: Bearer {api_key}
```

## Rate Limits
- **Public**: 100 req/min
- **Authenticated**: 1000 req/min

## Response Format

모든 응답은 JSON 형식:
```json
{
  "success": true,
  "data": {...},
  "timestamp": "2025-12-16T12:00:00Z"
}
```

오류 응답:
```json
{
  "success": false,
  "error": {
    "code": "ERROR_CODE",
    "message": "Error description"
  }
}
```

## WebSocket Protocol

실시간 업데이트를 위한 WebSocket:
```
ws://api.example.com/ws/rides/{rideId}
```

메시지 형식:
```json
{
  "type": "location_update",
  "data": {
    "latitude": 37.5665,
    "longitude": 126.9780,
    "eta": 300
  }
}
```

## 완전한 API 문서

OpenAPI 3.1.0 specification:
- **파일**: `openapi.yaml`
- **렌더링**: Swagger UI, Redoc, Stoplight 등 도구 사용

---

**Author**: Yeon Sam-Heum, Ph.D.  
**License**: MIT  
**弘益人間** - Benefit All Humanity
