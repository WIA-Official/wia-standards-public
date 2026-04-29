# 제5장: Phase 2 - API 인터페이스

## 시스템 간 연결의 표준

Phase 2는 언어 보존 시스템 간의 데이터 교환을 위한 표준화된 API를 정의합니다. RESTful 설계 원칙을 따르면서도 복잡한 쿼리를 위한 GraphQL 인터페이스를 함께 제공하여, 다양한 사용 사례를 지원합니다.

## API 설계 철학

### 핵심 원칙

| 원칙 | 설명 | 구현 방법 |
|------|------|----------|
| **REST First** | 단순한 작업은 REST로 | 표준 HTTP 메서드 사용 |
| **GraphQL 보완** | 복잡한 쿼리는 GraphQL로 | 유연한 데이터 조회 |
| **일관성** | 예측 가능한 인터페이스 | 명명 규칙 통일 |
| **버전 관리** | 하위 호환성 유지 | URL 경로에 버전 포함 |
| **보안 우선** | 모든 접근 인증 | OAuth 2.0 / JWT |

### 기본 URL 구조

```
프로덕션:  https://api.wia-lang.org/v1
스테이징:  https://api-staging.wia-lang.org/v1
개발:     http://localhost:3000/v1
```

## 인증 및 권한

### OAuth 2.0 흐름

```
┌──────────┐                              ┌──────────┐
│  Client  │                              │   Auth   │
│   App    │                              │  Server  │
└────┬─────┘                              └────┬─────┘
     │                                         │
     │  1. Authorization Request               │
     │────────────────────────────────────────>│
     │                                         │
     │  2. User Consent                        │
     │<────────────────────────────────────────│
     │                                         │
     │  3. Authorization Code                  │
     │────────────────────────────────────────>│
     │                                         │
     │  4. Access Token                        │
     │<────────────────────────────────────────│
     │                                         │
     │  5. API Request with Token              │
     │─────────────────────────────────────────>
```

### 인증 헤더

```http
Authorization: Bearer eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...
Content-Type: application/json
Accept: application/json
X-Request-ID: uuid-v4
```

### JWT 토큰 구조

```json
{
  "header": {
    "alg": "RS256",
    "typ": "JWT",
    "kid": "key-id-001"
  },
  "payload": {
    "sub": "user-12345",
    "iss": "https://auth.wia-lang.org",
    "aud": "https://api.wia-lang.org",
    "exp": 1735689600,
    "iat": 1735603200,
    "scopes": ["read:languages", "write:recordings", "admin:users"],
    "org": "org-001",
    "role": "researcher"
  }
}
```

### 권한 범위 (Scopes)

| 범위 | 설명 | 필요 역할 |
|------|------|----------|
| `read:languages` | 언어 정보 조회 | 모든 사용자 |
| `write:languages` | 언어 정보 수정 | 연구자 |
| `read:recordings` | 녹음 조회 | 인증 사용자 |
| `write:recordings` | 녹음 업로드 | 연구자 |
| `read:transcriptions` | 전사 조회 | 인증 사용자 |
| `write:transcriptions` | 전사 생성/수정 | 연구자 |
| `admin:*` | 관리 기능 | 관리자 |

## RESTful API 엔드포인트

### 언어 리소스

**언어 목록 조회**:
```http
GET /v1/languages?status=endangered&limit=20&offset=0

Response 200 OK:
{
  "data": [
    {
      "id": "ain",
      "name": {
        "english": "Ainu",
        "native": "アイヌ イタㇰ"
      },
      "endangerment": "critically_endangered",
      "speakers": {
        "native": 10
      },
      "links": {
        "self": "/v1/languages/ain",
        "recordings": "/v1/languages/ain/recordings"
      }
    }
  ],
  "pagination": {
    "total": 2500,
    "limit": 20,
    "offset": 0,
    "hasNext": true
  }
}
```

**단일 언어 조회**:
```http
GET /v1/languages/ain

Response 200 OK:
{
  "data": {
    "id": "ain",
    "name": {
      "english": "Ainu",
      "native": "アイヌ イタㇰ",
      "korean": "아이누어"
    },
    "iso639-3": "ain",
    "glottolog": "ainu1240",
    "family": "Language Isolate",
    "endangerment": {
      "status": "critically_endangered",
      "unescoLevel": 4
    },
    "speakers": {
      "native": 10,
      "l2": 300,
      "lastUpdated": "2025-01-01"
    },
    "regions": [
      {
        "country": "JP",
        "subdivision": "Hokkaido"
      }
    ],
    "resources": {
      "recordings": 150,
      "transcriptions": 120,
      "lexiconEntries": 5000
    }
  }
}
```

### 녹음 리소스

**녹음 업로드**:
```http
POST /v1/languages/ain/recordings
Content-Type: multipart/form-data

--boundary
Content-Disposition: form-data; name="file"; filename="story.flac"
Content-Type: audio/flac

[binary audio data]
--boundary
Content-Disposition: form-data; name="metadata"
Content-Type: application/json

{
  "type": "narrative",
  "speaker": {
    "id": "spk-001"
  },
  "session": {
    "date": "2025-01-01",
    "location": "Nibutani, Hokkaido"
  },
  "content": {
    "genre": "personal_narrative",
    "topic": "traditional_fishing"
  }
}
--boundary--

Response 201 Created:
{
  "data": {
    "id": "rec-20250101-ain-001",
    "status": "processing",
    "links": {
      "self": "/v1/recordings/rec-20250101-ain-001",
      "audio": "/v1/recordings/rec-20250101-ain-001/audio"
    }
  }
}
```

**녹음 목록 조회**:
```http
GET /v1/languages/ain/recordings?type=narrative&limit=10

Response 200 OK:
{
  "data": [
    {
      "id": "rec-20250101-ain-001",
      "type": "narrative",
      "duration": 325.5,
      "speaker": {
        "id": "spk-001",
        "age": 82
      },
      "created": "2025-01-01T10:00:00Z",
      "links": {
        "self": "/v1/recordings/rec-20250101-ain-001",
        "audio": "/v1/recordings/rec-20250101-ain-001/audio",
        "transcription": "/v1/recordings/rec-20250101-ain-001/transcription"
      }
    }
  ],
  "pagination": {
    "total": 150,
    "limit": 10,
    "offset": 0
  }
}
```

**오디오 스트리밍**:
```http
GET /v1/recordings/rec-20250101-ain-001/audio
Accept: audio/flac
Range: bytes=0-1048575

Response 206 Partial Content:
Content-Type: audio/flac
Content-Range: bytes 0-1048575/10485760
Accept-Ranges: bytes

[binary audio data]
```

### 전사 리소스

**전사 생성**:
```http
POST /v1/recordings/rec-20250101-ain-001/transcription

{
  "tiers": [
    {"name": "utterance", "language": "ain"},
    {"name": "translation", "language": "en"}
  ],
  "segments": [
    {
      "start": 0.0,
      "end": 3.5,
      "data": {
        "utterance": "ウコイキ オルンオシキ イタㇰ アン",
        "translation": "We have a language to speak to you"
      }
    }
  ]
}

Response 201 Created:
{
  "data": {
    "id": "tra-ain-001",
    "recording": "rec-20250101-ain-001",
    "segments": 1,
    "links": {
      "self": "/v1/transcriptions/tra-ain-001"
    }
  }
}
```

### 사전/어휘 리소스

**어휘 검색**:
```http
GET /v1/languages/ain/lexicon?q=kamui&pos=noun&limit=10

Response 200 OK:
{
  "data": [
    {
      "id": "lex-ain-001-0001",
      "headword": "カムイ",
      "phonetic": "/kamui/",
      "partOfSpeech": "noun",
      "definitions": [
        {
          "language": "en",
          "text": "god, deity, spirit"
        }
      ]
    }
  ]
}
```

**어휘 항목 추가**:
```http
POST /v1/languages/ain/lexicon

{
  "headword": "イタㇰ",
  "phonetic": "/itak/",
  "partOfSpeech": "noun",
  "definitions": [
    {
      "language": "en",
      "text": "language, word, speech"
    },
    {
      "language": "ko",
      "text": "언어, 말, 발화"
    }
  ],
  "examples": [
    {
      "sentence": "アイヌ イタㇰ",
      "translation": {
        "en": "Ainu language"
      }
    }
  ]
}

Response 201 Created
```

## GraphQL 인터페이스

### 스키마 개요

```graphql
type Query {
  language(code: String!): Language
  languages(
    status: EndangermentStatus
    family: String
    region: String
    limit: Int = 20
    offset: Int = 0
  ): LanguageConnection!

  recording(id: ID!): Recording
  recordings(
    language: String!
    type: RecordingType
    limit: Int = 20
  ): RecordingConnection!

  search(query: String!, types: [ResourceType!]): SearchResult!
}

type Mutation {
  createRecording(input: RecordingInput!): Recording!
  updateRecording(id: ID!, input: RecordingUpdateInput!): Recording!
  deleteRecording(id: ID!): Boolean!

  createTranscription(
    recordingId: ID!
    input: TranscriptionInput!
  ): Transcription!
}

type Language {
  id: ID!
  code: String!
  name: LanguageName!
  endangerment: Endangerment!
  speakers: SpeakerInfo!
  regions: [Region!]!
  recordings(limit: Int = 10): [Recording!]!
  lexicon(query: String, limit: Int = 20): [LexiconEntry!]!
}

type Recording {
  id: ID!
  language: Language!
  type: RecordingType!
  duration: Float!
  speaker: Speaker!
  session: Session!
  audioUrl: String!
  transcription: Transcription
  createdAt: DateTime!
}
```

### 복합 쿼리 예시

**언어와 관련 리소스 한 번에 조회**:
```graphql
query GetLanguageWithResources($code: String!) {
  language(code: $code) {
    id
    name {
      english
      native
    }
    endangerment {
      status
      unescoLevel
    }
    speakers {
      native
      lastUpdated
    }
    recordings(limit: 5) {
      id
      type
      duration
      speaker {
        age
        nativeStatus
      }
      transcription {
        id
        segments
      }
    }
    lexicon(limit: 10) {
      headword
      phonetic
      definitions {
        language
        text
      }
    }
  }
}
```

**변수**:
```json
{
  "code": "ain"
}
```

**복잡한 검색**:
```graphql
query SearchResources($query: String!) {
  search(query: $query, types: [RECORDING, LEXICON]) {
    totalCount
    results {
      ... on Recording {
        id
        type
        language {
          name {
            english
          }
        }
        duration
      }
      ... on LexiconEntry {
        id
        headword
        definitions {
          text
        }
      }
    }
  }
}
```

## 오류 처리

### 표준 오류 형식

```json
{
  "error": {
    "code": "VALIDATION_ERROR",
    "message": "The request body contains invalid data",
    "details": [
      {
        "field": "speaker.age",
        "issue": "must be a positive integer",
        "value": -5
      }
    ],
    "requestId": "req-12345-abcde",
    "timestamp": "2025-01-01T10:00:00Z",
    "documentation": "https://docs.wia-lang.org/errors/VALIDATION_ERROR"
  }
}
```

### HTTP 상태 코드

| 코드 | 의미 | 사용 상황 |
|------|------|----------|
| 200 | OK | 성공적인 GET, PUT, PATCH |
| 201 | Created | 성공적인 POST |
| 204 | No Content | 성공적인 DELETE |
| 400 | Bad Request | 잘못된 요청 형식 |
| 401 | Unauthorized | 인증 필요 |
| 403 | Forbidden | 권한 없음 |
| 404 | Not Found | 리소스 없음 |
| 409 | Conflict | 리소스 충돌 |
| 422 | Unprocessable | 유효성 검증 실패 |
| 429 | Too Many Requests | 속도 제한 초과 |
| 500 | Internal Error | 서버 오류 |

### 에러 코드 목록

| 에러 코드 | 설명 |
|----------|------|
| `AUTH_INVALID_TOKEN` | 유효하지 않은 토큰 |
| `AUTH_TOKEN_EXPIRED` | 만료된 토큰 |
| `AUTH_INSUFFICIENT_SCOPE` | 권한 범위 부족 |
| `LANG_NOT_FOUND` | 언어를 찾을 수 없음 |
| `RECORDING_NOT_FOUND` | 녹음을 찾을 수 없음 |
| `VALIDATION_ERROR` | 유효성 검증 실패 |
| `FILE_TOO_LARGE` | 파일 크기 초과 |
| `UNSUPPORTED_FORMAT` | 지원하지 않는 형식 |
| `RATE_LIMIT_EXCEEDED` | 요청 한도 초과 |

## 속도 제한

### 제한 정책

| 티어 | 요청/분 | 요청/일 | 업로드/일 |
|------|--------|--------|----------|
| Free | 60 | 1,000 | 100MB |
| Pro | 300 | 10,000 | 10GB |
| Enterprise | 1,000 | 무제한 | 100GB |
| Community | 100 | 5,000 | 1GB |

### 제한 헤더

```http
HTTP/1.1 200 OK
X-RateLimit-Limit: 60
X-RateLimit-Remaining: 45
X-RateLimit-Reset: 1735689600
X-RateLimit-Policy: "60;w=60"
```

### 제한 초과 응답

```http
HTTP/1.1 429 Too Many Requests
Retry-After: 30

{
  "error": {
    "code": "RATE_LIMIT_EXCEEDED",
    "message": "Rate limit exceeded. Please retry after 30 seconds.",
    "retryAfter": 30
  }
}
```

## SDK 사용법

### JavaScript/TypeScript

```typescript
import { WIALangClient } from '@wia/lang-sdk';

// 클라이언트 초기화
const client = new WIALangClient({
  apiKey: process.env.WIA_API_KEY,
  baseUrl: 'https://api.wia-lang.org/v1'
});

// 언어 조회
const language = await client.languages.get('ain');
console.log(language.name.english); // "Ainu"

// 녹음 업로드
const recording = await client.recordings.create({
  language: 'ain',
  file: fs.createReadStream('story.flac'),
  metadata: {
    type: 'narrative',
    speaker: { id: 'spk-001' }
  }
});

// 전사 조회
const transcription = await client.transcriptions.get(recording.id);

// 어휘 검색
const entries = await client.lexicon.search('ain', {
  query: 'kamui',
  pos: 'noun'
});
```

### Python

```python
from wia_lang import Client

# 클라이언트 초기화
client = Client(api_key=os.getenv('WIA_API_KEY'))

# 언어 조회
language = client.languages.get('ain')
print(language.name['english'])  # "Ainu"

# 녹음 업로드
with open('story.flac', 'rb') as f:
    recording = client.recordings.create(
        language='ain',
        file=f,
        metadata={
            'type': 'narrative',
            'speaker': {'id': 'spk-001'}
        }
    )

# 전사 생성
transcription = client.transcriptions.create(
    recording_id=recording.id,
    tiers=[
        {'name': 'utterance', 'language': 'ain'},
        {'name': 'translation', 'language': 'en'}
    ],
    segments=[
        {
            'start': 0.0,
            'end': 3.5,
            'data': {
                'utterance': 'ウコイキ オルンオシキ イタㇰ アン',
                'translation': 'We have a language to speak to you'
            }
        }
    ]
)

# 어휘 검색
entries = client.lexicon.search('ain', query='kamui', pos='noun')
```

### cURL 예시

```bash
# 인증 토큰 획득
curl -X POST https://auth.wia-lang.org/oauth/token \
  -d "grant_type=client_credentials" \
  -d "client_id=$CLIENT_ID" \
  -d "client_secret=$CLIENT_SECRET"

# 언어 목록 조회
curl -X GET "https://api.wia-lang.org/v1/languages?status=endangered" \
  -H "Authorization: Bearer $TOKEN"

# 녹음 업로드
curl -X POST "https://api.wia-lang.org/v1/languages/ain/recordings" \
  -H "Authorization: Bearer $TOKEN" \
  -F "file=@story.flac" \
  -F 'metadata={"type":"narrative","speaker":{"id":"spk-001"}}'
```

## 웹훅

### 웹훅 등록

```http
POST /v1/webhooks

{
  "url": "https://your-app.com/wia-webhook",
  "events": [
    "recording.created",
    "recording.processed",
    "transcription.created",
    "transcription.updated"
  ],
  "secret": "your-webhook-secret"
}

Response 201 Created:
{
  "data": {
    "id": "whk-12345",
    "url": "https://your-app.com/wia-webhook",
    "events": ["recording.created", ...],
    "status": "active"
  }
}
```

### 웹훅 페이로드

```json
{
  "id": "evt-67890",
  "type": "recording.processed",
  "timestamp": "2025-01-01T10:05:00Z",
  "data": {
    "recording": {
      "id": "rec-20250101-ain-001",
      "language": "ain",
      "status": "ready",
      "duration": 325.5
    }
  },
  "signature": "sha256=abc123..."
}
```

### 서명 검증

```python
import hmac
import hashlib

def verify_webhook(payload, signature, secret):
    expected = hmac.new(
        secret.encode(),
        payload.encode(),
        hashlib.sha256
    ).hexdigest()
    return hmac.compare_digest(f"sha256={expected}", signature)
```

---

## 핵심 요약

**주요 내용:**

1. **이중 API 설계**: RESTful API로 기본 CRUD, GraphQL로 복잡한 쿼리 지원

2. **OAuth 2.0 인증**: JWT 토큰 기반 인증, 세분화된 권한 범위(Scopes)

3. **RESTful 엔드포인트**: 언어, 녹음, 전사, 사전 리소스에 대한 표준 HTTP 인터페이스

4. **오류 처리**: 표준화된 오류 형식, 명확한 HTTP 상태 코드, 상세한 에러 코드

5. **다국어 SDK**: JavaScript, Python 등 주요 언어 지원, 직관적인 API

---

## 복습 문제

1. REST API와 GraphQL의 사용 사례 차이를 설명하시오.

2. OAuth 2.0 인증 흐름의 각 단계를 설명하시오.

3. 녹음 업로드 API의 요청 형식과 필수 메타데이터를 서술하시오.

4. GraphQL을 사용하여 언어와 관련 녹음을 한 번에 조회하는 쿼리를 작성하시오.

5. 속도 제한 정책의 티어별 차이와 제한 초과 시 대응 방법을 설명하시오.

6. 웹훅의 보안을 위한 서명 검증 과정을 설명하시오.

---

## 다음 장 미리보기

제6장에서는 Phase 3: 프로토콜을 다룹니다. 실시간 스트리밍, WebSocket 통신, 메시지 형식, 동기화 메커니즘, 그리고 보안 프로토콜에 대해 살펴봅니다.

---

**제5장 완료** | 예상 페이지: 18

[이전: 제4장 - 데이터 형식](./04-데이터-형식.md) | [다음: 제6장 - 프로토콜](./06-프로토콜.md)

---

© 2025 SmileStory Inc. / WIA
弘益人間 (홍익인간) · Benefit All Humanity
