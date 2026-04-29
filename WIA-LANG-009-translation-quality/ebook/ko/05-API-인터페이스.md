# 제5장: Phase 2 - API 인터페이스

## API 기반 품질 평가의 필요성

번역 품질 평가를 프로그래매틱하게 접근할 수 있는 API는 현대 번역 워크플로우의 필수 요소입니다. WIA-LANG-009 API는 다양한 시스템과 도구가 표준화된 방식으로 번역 품질을 평가하고, 데이터를 교환하며, 피드백을 수집할 수 있도록 합니다.

## API 아키텍처 개요

### RESTful 설계 원칙

WIA-LANG-009 API는 REST (Representational State Transfer) 아키텍처를 따릅니다.

**핵심 원칙**:
- **무상태성 (Stateless)**: 각 요청은 독립적
- **리소스 지향 (Resource-oriented)**: URL이 리소스를 나타냄
- **HTTP 메서드 활용**: GET, POST, PUT, DELETE
- **JSON 응답**: 일관된 JSON 형식

### API 엔드포인트 구조

```
https://api.wia-lang.org/v1/
├── /assess              # 품질 평가
├── /metrics             # 메트릭 계산
├── /annotate            # 오류 주석
├── /termbases           # 용어 관리
├── /qe                  # 품질 추정
└── /reports             # 리포트 생성
```

## 인증 및 권한

### API 키 기반 인증

```http
GET /api/v1/assess HTTP/1.1
Host: api.wia-lang.org
Authorization: Bearer wia_live_sk_1234567890abcdef
Content-Type: application/json
```

### JWT 토큰 인증

```json
{
  "auth": {
    "method": "JWT",
    "token_endpoint": "https://api.wia-lang.org/v1/auth/token",
    "request": {
      "grant_type": "client_credentials",
      "client_id": "your-client-id",
      "client_secret": "your-client-secret"
    },
    "response": {
      "access_token": "eyJhbGciOiJIUzI1NiIs...",
      "token_type": "Bearer",
      "expires_in": 3600
    }
  }
}
```

### 권한 레벨

| 레벨 | 설명 | 허용 작업 |
|------|------|----------|
| **read** | 읽기 전용 | GET 요청만 |
| **write** | 쓰기 가능 | GET, POST |
| **admin** | 관리자 | 모든 메서드 |
| **eval** | 평가자 | 평가 및 주석 작업 |

## 품질 평가 API

### POST /api/v1/assess

번역 품질을 평가합니다.

**요청**:
```http
POST /api/v1/assess HTTP/1.1
Host: api.wia-lang.org
Authorization: Bearer wia_live_sk_1234567890abcdef
Content-Type: application/json

{
  "source_language": "en",
  "target_language": "ko",
  "domain": "technical",
  "segments": [
    {
      "id": "seg-001",
      "source": "Click the Save button to save your changes.",
      "target": "변경 사항을 저장하려면 저장 버튼을 클릭하세요."
    },
    {
      "id": "seg-002",
      "source": "Enter your password.",
      "target": "암호를 입력하세요."
    }
  ],
  "metrics": ["bleu", "comet", "chrf"],
  "references": [
    {
      "id": "seg-001",
      "text": "변경 사항을 저장하려면 저장 버튼을 클릭하십시오."
    },
    {
      "id": "seg-002",
      "text": "비밀번호를 입력하세요."
    }
  ],
  "options": {
    "include_word_level": false,
    "quality_estimation": true
  }
}
```

**응답**:
```http
HTTP/1.1 200 OK
Content-Type: application/json

{
  "assessment_id": "assess-20250101-001",
  "status": "completed",
  "created_at": "2025-01-01T14:30:00Z",
  "processing_time_ms": 1250,
  "overall_quality": {
    "score": 87.5,
    "grade": "Good",
    "confidence": 0.89
  },
  "segments": [
    {
      "id": "seg-001",
      "quality": {
        "overall_score": 92.0,
        "dimensions": {
          "accuracy": 0.95,
          "fluency": 0.93,
          "consistency": 1.00,
          "adequacy": 0.90,
          "format": 1.00
        },
        "metrics": {
          "bleu": 0.78,
          "comet": 0.88,
          "chrf": 0.82
        },
        "qe": {
          "hter_prediction": 0.12,
          "quality_class": "good"
        }
      }
    },
    {
      "id": "seg-002",
      "quality": {
        "overall_score": 83.0,
        "dimensions": {
          "accuracy": 0.80,
          "fluency": 0.88,
          "consistency": 1.00,
          "adequacy": 0.85,
          "format": 1.00
        },
        "metrics": {
          "bleu": 0.45,
          "comet": 0.75,
          "chrf": 0.68
        },
        "qe": {
          "hter_prediction": 0.25,
          "quality_class": "acceptable"
        }
      }
    }
  ]
}
```

### GET /api/v1/assess/{assessment_id}

저장된 평가 결과를 조회합니다.

**요청**:
```http
GET /api/v1/assess/assess-20250101-001 HTTP/1.1
Host: api.wia-lang.org
Authorization: Bearer wia_live_sk_1234567890abcdef
```

**응답**:
```json
{
  "assessment_id": "assess-20250101-001",
  "status": "completed",
  "document": {
    "id": "doc-001",
    "metadata": {...},
    "quality_summary": {...},
    "segments": [...]
  }
}
```

## 메트릭 계산 API

### POST /api/v1/metrics/calculate

특정 메트릭을 계산합니다.

**요청**:
```http
POST /api/v1/metrics/calculate HTTP/1.1
Host: api.wia-lang.org
Authorization: Bearer wia_live_sk_1234567890abcdef
Content-Type: application/json

{
  "metric": "comet",
  "model": "Unbabel/wmt22-comet-da",
  "source": "Click the Save button.",
  "target": "저장 버튼을 클릭하세요.",
  "reference": "저장 버튼을 클릭하십시오."
}
```

**응답**:
```json
{
  "metric": "comet",
  "model": "Unbabel/wmt22-comet-da",
  "score": 0.8845,
  "details": {
    "model_type": "regression",
    "reference_based": true
  },
  "computed_at": "2025-01-01T14:32:00Z"
}
```

### GET /api/v1/metrics/supported

지원되는 메트릭 목록을 조회합니다.

**응답**:
```json
{
  "metrics": [
    {
      "name": "bleu",
      "type": "automatic",
      "reference_required": true,
      "languages": "all",
      "speed": "fast"
    },
    {
      "name": "comet",
      "type": "neural",
      "reference_required": true,
      "languages": ["en", "ko", "ja", "zh", "es", "de", "fr"],
      "models": [
        "Unbabel/wmt22-comet-da",
        "Unbabel/wmt21-comet-qe-mqm"
      ],
      "speed": "medium"
    },
    {
      "name": "chrf",
      "type": "automatic",
      "reference_required": true,
      "languages": "all",
      "speed": "fast"
    }
  ]
}
```

## 오류 주석 API

### POST /api/v1/annotate

오류를 주석합니다.

**요청**:
```json
{
  "segment_id": "seg-001",
  "source": "The quarterly report is due next week.",
  "target": "월간 보고서는 다음 주에 제출해야 합니다.",
  "annotations": [
    {
      "type": "accuracy",
      "subtype": "mistranslation",
      "severity": "major",
      "position": {
        "start": 0,
        "end": 8
      },
      "source_text": "quarterly",
      "target_text": "월간",
      "correction": "분기",
      "description": "quarterly를 월간으로 잘못 번역",
      "annotator": "annotator-001"
    }
  ]
}
```

**응답**:
```json
{
  "annotation_id": "annot-20250101-001",
  "status": "saved",
  "created_at": "2025-01-01T14:35:00Z",
  "quality_impact": {
    "before": 85.0,
    "after": 75.0,
    "delta": -10.0
  }
}
```

### GET /api/v1/annotate/{segment_id}

세그먼트의 주석을 조회합니다.

**응답**:
```json
{
  "segment_id": "seg-001",
  "annotations": [
    {
      "id": "annot-001",
      "type": "accuracy",
      "severity": "major",
      "annotator": "annotator-001",
      "created_at": "2025-01-01T14:35:00Z"
    }
  ],
  "annotation_count": 1,
  "inter_annotator_agreement": null
}
```

## 품질 추정 (QE) API

### POST /api/v1/qe/predict

참조 번역 없이 품질을 예측합니다.

**요청**:
```json
{
  "source_language": "en",
  "target_language": "ko",
  "segments": [
    {
      "id": "seg-001",
      "source": "Click the Save button.",
      "target": "저장 버튼을 클릭하세요."
    }
  ],
  "model": "OpenKiwi-XLM-R",
  "prediction_level": ["sentence", "word"]
}
```

**응답**:
```json
{
  "predictions": [
    {
      "segment_id": "seg-001",
      "sentence_level": {
        "hter": 0.08,
        "da_score": 85.2,
        "quality_class": "good",
        "confidence": 0.91
      },
      "word_level": {
        "tags": [
          {"word": "저장", "tag": "OK", "probability": 0.96},
          {"word": "버튼을", "tag": "OK", "probability": 0.94},
          {"word": "클릭하세요", "tag": "OK", "probability": 0.93}
        ]
      }
    }
  ],
  "model_info": {
    "name": "OpenKiwi-XLM-R",
    "version": "2.1.0"
  }
}
```

## 용어 관리 API

### POST /api/v1/termbases

새 용어집을 생성합니다.

**요청**:
```json
{
  "name": "Software Terminology",
  "source_language": "en",
  "target_languages": ["ko", "ja"],
  "domain": "software",
  "entries": [
    {
      "source_term": "user interface",
      "targets": [
        {
          "language": "ko",
          "term": "사용자 인터페이스",
          "status": "approved"
        }
      ]
    }
  ]
}
```

**응답**:
```json
{
  "termbase_id": "tb-20250101-001",
  "status": "created",
  "entry_count": 1,
  "created_at": "2025-01-01T15:00:00Z"
}
```

### POST /api/v1/termbases/{id}/check

용어 일관성을 검증합니다.

**요청**:
```json
{
  "segments": [
    {
      "id": "seg-001",
      "source": "The user interface is intuitive.",
      "target": "유저 인터페이스가 직관적입니다."
    }
  ]
}
```

**응답**:
```json
{
  "consistency_check": {
    "violations": [
      {
        "segment_id": "seg-001",
        "source_term": "user interface",
        "approved_translation": "사용자 인터페이스",
        "used_translation": "유저 인터페이스",
        "severity": "minor"
      }
    ],
    "consistency_rate": 0.0,
    "total_terms_checked": 1
  }
}
```

## 리포트 생성 API

### POST /api/v1/reports/generate

품질 리포트를 생성합니다.

**요청**:
```json
{
  "assessment_id": "assess-20250101-001",
  "format": "html",
  "sections": [
    "executive_summary",
    "detailed_metrics",
    "error_analysis",
    "recommendations"
  ],
  "language": "ko"
}
```

**응답**:
```json
{
  "report_id": "report-20250101-001",
  "status": "generated",
  "download_url": "https://api.wia-lang.org/v1/reports/report-20250101-001/download",
  "expires_at": "2025-01-08T15:00:00Z"
}
```

## GraphQL API

### GraphQL 스키마

```graphql
type Query {
  assessment(id: ID!): Assessment
  assessments(filter: AssessmentFilter, limit: Int): [Assessment]
  metrics(names: [String!]!): [MetricInfo]
  termbase(id: ID!): Termbase
}

type Mutation {
  createAssessment(input: AssessmentInput!): Assessment
  annotateError(input: ErrorAnnotationInput!): Annotation
  createTermbase(input: TermbaseInput!): Termbase
}

type Assessment {
  id: ID!
  status: AssessmentStatus!
  overallQuality: QualityScore!
  segments: [Segment!]!
  createdAt: DateTime!
}

type Segment {
  id: ID!
  source: String!
  target: String!
  quality: SegmentQuality!
  annotations: [Annotation!]!
}

type SegmentQuality {
  overallScore: Float!
  dimensions: QualityDimensions!
  metrics: [MetricResult!]!
}

type QualityDimensions {
  accuracy: Float!
  fluency: Float!
  consistency: Float!
  adequacy: Float!
  format: Float!
}
```

### GraphQL 쿼리 예시

**평가 조회**:
```graphql
query GetAssessment {
  assessment(id: "assess-20250101-001") {
    id
    status
    overallQuality {
      score
      grade
    }
    segments {
      id
      source
      target
      quality {
        overallScore
        dimensions {
          accuracy
          fluency
        }
        metrics {
          name
          score
        }
      }
    }
  }
}
```

**평가 생성**:
```graphql
mutation CreateAssessment {
  createAssessment(input: {
    sourceLanguage: "en"
    targetLanguage: "ko"
    domain: TECHNICAL
    segments: [
      {
        id: "seg-001"
        source: "Click Save"
        target: "저장 클릭"
      }
    ]
    metrics: [BLEU, COMET]
  }) {
    id
    status
    overallQuality {
      score
    }
  }
}
```

## SDK 예시

### Python SDK

**설치**:
```bash
pip install wia-lang-009
```

**사용 예시**:
```python
from wia_lang_009 import WIAQualityClient

# 클라이언트 초기화
client = WIAQualityClient(api_key="wia_live_sk_1234567890abcdef")

# 품질 평가
assessment = client.assess(
    source_language="en",
    target_language="ko",
    domain="technical",
    segments=[
        {
            "id": "seg-001",
            "source": "Click the Save button.",
            "target": "저장 버튼을 클릭하세요."
        }
    ],
    metrics=["bleu", "comet", "chrf"]
)

print(f"Overall Score: {assessment.overall_quality.score}")
print(f"Grade: {assessment.overall_quality.grade}")

# 세그먼트별 품질
for segment in assessment.segments:
    print(f"\nSegment {segment.id}:")
    print(f"  Score: {segment.quality.overall_score}")
    print(f"  Accuracy: {segment.quality.dimensions.accuracy}")
    print(f"  Fluency: {segment.quality.dimensions.fluency}")

# 오류 주석
annotation = client.annotate(
    segment_id="seg-001",
    error_type="accuracy",
    subtype="mistranslation",
    severity="major",
    position={"start": 0, "end": 5},
    description="용어 오역"
)

# 품질 추정 (QE)
qe_result = client.quality_estimation(
    source_language="en",
    target_language="ko",
    segments=[
        {
            "id": "seg-001",
            "source": "Save file",
            "target": "파일 저장"
        }
    ]
)

print(f"QE Prediction: {qe_result.predictions[0].sentence_level.da_score}")
```

### TypeScript SDK

**설치**:
```bash
npm install @wia/lang-009
```

**사용 예시**:
```typescript
import { WIAQualityClient } from '@wia/lang-009';

// 클라이언트 초기화
const client = new WIAQualityClient({
  apiKey: 'wia_live_sk_1234567890abcdef'
});

// 품질 평가
const assessment = await client.assess({
  sourceLanguage: 'en',
  targetLanguage: 'ko',
  domain: 'technical',
  segments: [
    {
      id: 'seg-001',
      source: 'Click the Save button.',
      target: '저장 버튼을 클릭하세요.'
    }
  ],
  metrics: ['bleu', 'comet', 'chrf']
});

console.log(`Overall Score: ${assessment.overallQuality.score}`);
console.log(`Grade: ${assessment.overallQuality.grade}`);

// 스트림 평가 (대용량)
const stream = await client.assessStream({
  sourceLanguage: 'en',
  targetLanguage: 'ko',
  segments: largeSegmentArray
});

for await (const result of stream) {
  console.log(`Segment ${result.id}: ${result.quality.overallScore}`);
}

// 용어 검증
const termCheck = await client.checkTerminology({
  termbaseId: 'tb-001',
  segments: [
    {
      id: 'seg-001',
      source: 'user interface',
      target: '유저 인터페이스'
    }
  ]
});

console.log(`Violations: ${termCheck.violations.length}`);
```

### Java SDK

**Maven 의존성**:
```xml
<dependency>
  <groupId>org.wia</groupId>
  <artifactId>lang-009</artifactId>
  <version>1.0.0</version>
</dependency>
```

**사용 예시**:
```java
import org.wia.lang009.WIAQualityClient;
import org.wia.lang009.model.*;

public class QualityAssessmentExample {
    public static void main(String[] args) {
        // 클라이언트 초기화
        WIAQualityClient client = new WIAQualityClient.Builder()
            .apiKey("wia_live_sk_1234567890abcdef")
            .build();

        // 평가 요청 생성
        AssessmentRequest request = AssessmentRequest.builder()
            .sourceLanguage("en")
            .targetLanguage("ko")
            .domain(Domain.TECHNICAL)
            .segment(Segment.builder()
                .id("seg-001")
                .source("Click the Save button.")
                .target("저장 버튼을 클릭하세요.")
                .build())
            .metrics(Arrays.asList("bleu", "comet"))
            .build();

        // 평가 수행
        Assessment assessment = client.assess(request);

        System.out.println("Overall Score: " +
            assessment.getOverallQuality().getScore());

        // 세그먼트 품질
        for (SegmentQuality sq : assessment.getSegments()) {
            System.out.println("Segment: " + sq.getId());
            System.out.println("  Score: " + sq.getQuality().getOverallScore());
        }
    }
}
```

## 오류 처리

### 표준 오류 응답

```json
{
  "error": {
    "code": "invalid_parameter",
    "message": "Invalid source language code",
    "details": {
      "field": "source_language",
      "provided": "eng",
      "expected": "BCP 47 format (e.g., 'en', 'en-US')"
    },
    "request_id": "req-20250101-001",
    "timestamp": "2025-01-01T14:30:00Z"
  }
}
```

### HTTP 상태 코드

| 코드 | 의미 | 사용 예시 |
|------|------|----------|
| 200 | OK | 성공적인 요청 |
| 201 | Created | 리소스 생성 성공 |
| 400 | Bad Request | 잘못된 요청 파라미터 |
| 401 | Unauthorized | 인증 실패 |
| 403 | Forbidden | 권한 부족 |
| 404 | Not Found | 리소스를 찾을 수 없음 |
| 429 | Too Many Requests | 속도 제한 초과 |
| 500 | Internal Server Error | 서버 오류 |
| 503 | Service Unavailable | 서비스 일시 중단 |

### 재시도 정책

```python
import time
from wia_lang_009 import WIAQualityClient
from wia_lang_009.exceptions import RateLimitError

client = WIAQualityClient(api_key="your-api-key")

max_retries = 3
retry_delay = 1  # 초

for attempt in range(max_retries):
    try:
        assessment = client.assess(...)
        break
    except RateLimitError as e:
        if attempt < max_retries - 1:
            wait_time = retry_delay * (2 ** attempt)  # 지수 백오프
            print(f"Rate limited. Retrying in {wait_time}s...")
            time.sleep(wait_time)
        else:
            raise
```

## 속도 제한 (Rate Limiting)

### 제한 정책

| 플랜 | 요청/분 | 요청/일 | 동시 요청 |
|------|---------|---------|----------|
| Free | 10 | 1,000 | 1 |
| Basic | 60 | 10,000 | 5 |
| Pro | 600 | 100,000 | 20 |
| Enterprise | 무제한 | 무제한 | 100 |

### 응답 헤더

```http
HTTP/1.1 200 OK
X-RateLimit-Limit: 60
X-RateLimit-Remaining: 45
X-RateLimit-Reset: 1704124800
```

---

## 핵심 요약

**API 인터페이스:**

1. **RESTful 설계**: 표준 HTTP 메서드와 리소스 지향 URL

2. **다양한 엔드포인트**: 평가, 메트릭, 주석, QE, 용어, 리포트

3. **GraphQL 지원**: 유연한 쿼리와 효율적인 데이터 페칭

4. **다중 언어 SDK**: Python, TypeScript, Java 지원

5. **강력한 오류 처리**: 명확한 오류 메시지와 재시도 로직

6. **속도 제한**: 공정한 사용을 위한 플랜별 제한

---

## 복습 문제

1. WIA-LANG-009 API의 RESTful 설계 원칙과 주요 엔드포인트를 설명하시오.

2. 품질 평가 API의 요청/응답 구조를 설명하고, 주요 필드의 역할을 서술하시오.

3. GraphQL API가 RESTful API에 비해 가지는 장점을 번역 품질 평가 맥락에서 설명하시오.

4. Python SDK를 사용한 품질 평가 및 오류 주석 과정을 코드로 작성하시오.

5. API 오류 처리의 모범 사례와 재시도 정책의 필요성을 논하시오.

6. 속도 제한이 필요한 이유와 플랜별 제한의 합리성을 설명하시오.

---

## 다음 장 미리보기

제6장에서는 Phase 3: 프로토콜을 다룹니다. 번역 전/중/후 품질 보증 프로세스, 평가자 가이드라인, 품질 게이트, 피드백 루프, 그리고 지속적 품질 개선 워크플로우를 상세히 살펴봅니다.

---

**제5장 완료** | 예상 페이지: 18

[이전: 제4장 - 데이터 형식](./04-데이터-형식.md) | [다음: 제6장 - 프로토콜](./06-프로토콜.md)

---

© 2025 SmileStory Inc. / WIA
弘益人間 (홍익인간) · Benefit All Humanity
