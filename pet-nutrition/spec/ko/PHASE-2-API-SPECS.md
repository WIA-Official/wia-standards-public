# WIA-PET-009: 2단계 - API 사양

**버전:** 1.0 | **상태:** 최종 | **최종 업데이트:** 2025-12-25

## REST API 엔드포인트

### 인증
```
POST /api/v1/auth/token
Content-Type: application/json

{
  "grant_type": "client_credentials",
  "client_id": "string",
  "client_secret": "string",
  "scope": "pet.read pet.write nutrition.read"
}
```

### 반려동물 관리
```
GET    /api/v1/pets/{petId}           # 반려동물 조회
POST   /api/v1/pets                   # 반려동물 생성
PUT    /api/v1/pets/{petId}           # 반려동물 수정
DELETE /api/v1/pets/{petId}           # 반려동물 삭제
GET    /api/v1/pets?ownerId={id}     # 보호자별 반려동물 목록
```

### 영양 요구사항
```
GET    /api/v1/pets/{petId}/nutrition/requirements
PUT    /api/v1/pets/{petId}/nutrition/requirements
POST   /api/v1/nutrition/calculate
```

### 식단 계획
```
GET    /api/v1/pets/{petId}/diet-plans
POST   /api/v1/pets/{petId}/diet-plans
GET    /api/v1/diet-plans/{planId}
PUT    /api/v1/diet-plans/{planId}
DELETE /api/v1/diet-plans/{planId}
```

### 급식 로그
```
GET    /api/v1/pets/{petId}/feeding-logs?startDate={date}&endDate={date}
POST   /api/v1/feeding-logs
GET    /api/v1/feeding-logs/{logId}
```

### 알레르기 관리
```
GET    /api/v1/pets/{petId}/allergies
POST   /api/v1/pets/{petId}/allergies
DELETE /api/v1/pets/{petId}/allergies/{allergyId}
```

### 사료 제품
```
GET    /api/v1/products
GET    /api/v1/products/{productId}
POST   /api/v1/products/search
GET    /api/v1/products/{productId}/allergen-check?petId={petId}
```

### 체중 추적
```
GET    /api/v1/pets/{petId}/weight-history
POST   /api/v1/pets/{petId}/weight-measurements
```

## HTTP 상태 코드

| 코드 | 의미 | 사용 |
|------|------|------|
| 200 | OK | 성공적인 GET, PUT |
| 201 | Created | 성공적인 POST |
| 204 | No Content | 성공적인 DELETE |
| 400 | Bad Request | 잘못된 JSON, 필수 필드 누락 |
| 401 | Unauthorized | 토큰 누락/무효 |
| 403 | Forbidden | 권한 부족 |
| 404 | Not Found | 리소스 없음 |
| 409 | Conflict | 중복 리소스 |
| 422 | Validation Error | 잘못된 데이터 값 |
| 429 | Rate Limit Exceeded | 요청 과다 |
| 500 | Internal Server Error | 서버 오류 |
| 503 | Service Unavailable | 유지보수/중단 |

## 속도 제한

**표준 등급:** 시간당 1,000 요청 / API 키  
**프리미엄 등급:** 시간당 10,000 요청 / API 키  
**엔터프라이즈 등급:** 맞춤형 제한

속도 제한 헤더:
```
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 847
X-RateLimit-Reset: 1640000000
```

## 페이지네이션

목록 엔드포인트의 경우:
```
GET /api/v1/pets?page=2&limit=50

응답:
{
  "data": [...],
  "pagination": {
    "page": 2,
    "limit": 50,
    "total": 247,
    "totalPages": 5
  }
}
```

## 오류 응답 형식

```json
{
  "error": {
    "code": "VALIDATION_ERROR",
    "message": "반려동물 프로필 유효성 검사 실패",
    "details": [
      {
        "field": "weight.current",
        "issue": "양수여야 합니다",
        "value": -5.2
      }
    ],
    "timestamp": "2025-12-25T14:30:00Z",
    "requestId": "req-12345-abcdef"
  }
}
```

## 버전 관리

URL 기반: `/api/v1/`, `/api/v2/`  
24개월 동안 N-1 버전 지원

---

**弘益人間 · 널리 인간을 이롭게 하라**  
© 2025 WIA | WIA-PET-009 v1.0 단계 2
