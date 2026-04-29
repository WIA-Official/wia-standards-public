# 제5장: API 인터페이스

## Phase 2: 통일된 API 설계

API (Application Programming Interface)는 언어 AI 시스템에 접근하는 관문입니다. WIA-LANG-005 Phase 2는 일관되고 직관적이며 확장 가능한 API 인터페이스를 정의하여, 개발자들이 쉽게 저자원 언어 AI 기능을 애플리케이션에 통합할 수 있도록 합니다.

## RESTful API 설계

### API 기본 구조

**Base URL**:
```
https://api.wia-lang-005.org/v1
```

**일반 요청 형식**:
```http
POST /v1/{task}
Content-Type: application/json
Authorization: Bearer {api_key}
X-WIA-Version: 1.0.0

{
  "language": "ain",
  "input": { },
  "options": { }
}
```

**일반 응답 형식**:
```json
{
  "status": "success",
  "data": { },
  "metadata": {
    "request_id": "req_abc123",
    "timestamp": "2025-03-15T10:30:00Z",
    "latency_ms": 125,
    "model_version": "ain-bert-base-v1.0"
  },
  "usage": {
    "characters": 50,
    "tokens": 12,
    "credits_used": 1
  }
}
```

### 1. 번역 API (Translation)

**엔드포인트**: `POST /v1/translate`

**요청**:
```json
{
  "source_language": "ain",
  "target_language": "ja",
  "text": "イランカラプテ",
  "options": {
    "formality": "polite",
    "domain": "general",
    "alternatives": 3,
    "preserve_formatting": true
  }
}
```

**응답**:
```json
{
  "status": "success",
  "data": {
    "translation": "こんにちは",
    "confidence": 0.92,
    "alternatives": [
      {
        "text": "ご挨拶申し上げます",
        "confidence": 0.85,
        "formality": "very_polite"
      },
      {
        "text": "やあ",
        "confidence": 0.73,
        "formality": "casual"
      }
    ],
    "detected_language": "ain",
    "language_confidence": 0.98
  },
  "metadata": {
    "model": "ain-ja-nmt-v1.2",
    "request_id": "tr_001",
    "latency_ms": 87
  }
}
```

**배치 번역**:
```json
POST /v1/translate/batch

{
  "source_language": "ain",
  "target_language": "ja",
  "texts": [
    "イランカラプテ",
    "イヤイライケレ",
    "カムイモシㇼ"
  ]
}

// 응답
{
  "status": "success",
  "data": {
    "translations": [
      {"text": "こんにちは", "confidence": 0.92},
      {"text": "ありがとうございます", "confidence": 0.88},
      {"text": "神々の国", "confidence": 0.85}
    ]
  }
}
```

### 2. 음성 인식 API (ASR)

**엔드포인트**: `POST /v1/asr`

**요청** (multipart/form-data):
```http
POST /v1/asr
Content-Type: multipart/form-data

------WebKitFormBoundary
Content-Disposition: form-data; name="audio"; filename="recording.wav"
Content-Type: audio/wav

[binary audio data]
------WebKitFormBoundary
Content-Disposition: form-data; name="language"

ain
------WebKitFormBoundary
Content-Disposition: form-data; name="options"

{
  "model": "ain-asr-v2.0",
  "return_timestamps": true,
  "language_detection": true,
  "punctuation": true
}
------WebKitFormBoundary--
```

**응답**:
```json
{
  "status": "success",
  "data": {
    "transcription": "イランカラプテ、私の名前はアイヌです。",
    "confidence": 0.88,
    "language": "ain",
    "language_confidence": 0.95,
    "segments": [
      {
        "text": "イランカラプテ",
        "start": 0.0,
        "end": 1.2,
        "confidence": 0.92
      },
      {
        "text": "私の名前はアイヌです",
        "start": 1.5,
        "end": 3.8,
        "confidence": 0.85
      }
    ],
    "words": [
      {"word": "イランカラプテ", "start": 0.0, "end": 1.2, "confidence": 0.92},
      {"word": "私", "start": 1.5, "end": 1.7, "confidence": 0.88}
    ]
  },
  "audio_info": {
    "duration": 3.8,
    "format": "wav",
    "sample_rate": 16000,
    "channels": 1
  }
}
```

**실시간 스트리밍 ASR**:
```javascript
// WebSocket 연결
const ws = new WebSocket('wss://api.wia-lang-005.org/v1/asr/stream');

ws.on('open', () => {
  // 설정 전송
  ws.send(JSON.stringify({
    type: 'config',
    language: 'ain',
    sample_rate: 16000
  }));

  // 오디오 스트리밍 시작
  audioStream.on('data', (chunk) => {
    ws.send(chunk);
  });
});

ws.on('message', (data) => {
  const result = JSON.parse(data);
  if (result.type === 'partial') {
    console.log('Partial:', result.text);
  } else if (result.type === 'final') {
    console.log('Final:', result.text);
  }
});
```

### 3. 텍스트 음성 변환 API (TTS)

**엔드포인트**: `POST /v1/tts`

**요청**:
```json
{
  "text": "イランカラプテ",
  "language": "ain",
  "voice": {
    "id": "ain-female-01",
    "gender": "female",
    "age": "adult",
    "style": "neutral"
  },
  "options": {
    "speed": 1.0,
    "pitch": 1.0,
    "volume": 1.0,
    "format": "mp3",
    "sample_rate": 22050
  }
}
```

**응답**:
```json
{
  "status": "success",
  "data": {
    "audio": "data:audio/mp3;base64,SUQzBAAAAAAAI1RTU0UAAAA...",
    "duration": 2.5,
    "format": "mp3",
    "sample_rate": 22050,
    "phonemes": "i r a N k a r a p u t e",
    "visemes": [
      {"viseme": "i", "start": 0.0, "end": 0.15},
      {"viseme": "r", "start": 0.15, "end": 0.35}
    ]
  },
  "metadata": {
    "voice_id": "ain-female-01",
    "model": "ain-tacotron2-v1.0"
  }
}
```

**음성 목록 조회**:
```json
GET /v1/tts/voices?language=ain

// 응답
{
  "status": "success",
  "data": {
    "voices": [
      {
        "id": "ain-female-01",
        "name": "Reiko",
        "language": "ain",
        "gender": "female",
        "age_group": "adult",
        "description": "Native Ainu speaker from Biratori",
        "sample_url": "https://samples.wia.org/ain-female-01.mp3",
        "styles": ["neutral", "storytelling", "conversational"]
      },
      {
        "id": "ain-male-01",
        "name": "Takeshi",
        "language": "ain",
        "gender": "male",
        "age_group": "elder",
        "description": "Elder speaker, traditional storytelling style"
      }
    ]
  }
}
```

### 4. 언어 모델 API (Language Modeling)

**텍스트 완성**:
```json
POST /v1/complete

{
  "prompt": "アイヌモシㇼは",
  "language": "ain",
  "options": {
    "max_tokens": 50,
    "temperature": 0.7,
    "top_p": 0.9,
    "stop_sequences": ["。", "\n"],
    "num_completions": 3
  }
}

// 응답
{
  "status": "success",
  "data": {
    "completions": [
      {
        "text": "カムイモシㇼとも呼ばれ、アイヌの人々が住む大地を意味する。",
        "probability": 0.85,
        "tokens": 28
      },
      {
        "text": "アイヌの人々の故郷であり、神々が見守る土地である。",
        "probability": 0.78,
        "tokens": 25
      }
    ]
  }
}
```

**텍스트 임베딩**:
```json
POST /v1/embeddings

{
  "texts": [
    "イランカラプテ",
    "こんにちは"
  ],
  "language": "ain",
  "model": "ain-bert-base"
}

// 응답
{
  "status": "success",
  "data": {
    "embeddings": [
      [0.123, -0.456, 0.789, ...],  // 768-dim vector
      [0.234, -0.567, 0.890, ...]
    ],
    "model": "ain-bert-base",
    "dimensions": 768
  }
}
```

### 5. 개체명 인식 API (NER)

**엔드포인트**: `POST /v1/ner`

**요청**:
```json
{
  "text": "シㇼカムイはカムイモシㇼの山々に住んでいる。",
  "language": "ain",
  "entity_types": ["PERSON", "LOCATION", "DEITY", "ANIMAL_SPIRIT"]
}
```

**응답**:
```json
{
  "status": "success",
  "data": {
    "entities": [
      {
        "text": "シㇼカムイ",
        "type": "DEITY",
        "start": 0,
        "end": 5,
        "confidence": 0.98,
        "gloss": "mountain god; bear deity",
        "metadata": {
          "cultural_significance": "sacred",
          "alternative_names": ["熊の神", "Kim-un-kamuy"]
        }
      },
      {
        "text": "カムイモシㇼ",
        "type": "LOCATION",
        "start": 6,
        "end": 12,
        "confidence": 0.95,
        "gloss": "land of the gods"
      }
    ],
    "relations": [
      {
        "type": "RESIDES_IN",
        "subject": "シㇼカムイ",
        "object": "カムイモシㇼ",
        "confidence": 0.87
      }
    ]
  }
}
```

## GraphQL API

### 스키마 정의

```graphql
type Query {
  # 번역 조회
  translate(
    text: String!
    sourceLang: String!
    targetLang: String!
    options: TranslateOptions
  ): TranslationResult!

  # 언어 정보
  language(code: String!): Language
  languages(filter: LanguageFilter): [Language!]!

  # 모델 정보
  model(id: String!): Model
  models(task: TaskType, language: String): [Model!]!

  # 음성 목록
  voices(language: String): [Voice!]!
}

type Mutation {
  # 음성 인식
  transcribeAudio(
    audio: Upload!
    language: String!
    options: ASROptions
  ): ASRResult!

  # 음성 합성
  synthesizeSpeech(
    text: String!
    language: String!
    voice: String
    options: TTSOptions
  ): TTSResult!

  # 피드백 제출
  submitFeedback(
    requestId: String!
    rating: Int!
    comment: String
  ): Feedback!
}

type Language {
  code: String!
  name: String!
  nativeName: String
  family: String
  endangerment: EndangermentLevel!
  speakerCount: Int
  supportedTasks: [TaskType!]!
  models: [Model!]!
}

type TranslationResult {
  translation: String!
  confidence: Float!
  alternatives: [Alternative!]
  metadata: Metadata!
}

type Model {
  id: String!
  name: String!
  task: TaskType!
  language: String!
  version: String!
  accuracy: Float
  parameters: Int
  createdAt: DateTime!
}

enum TaskType {
  TRANSLATION
  ASR
  TTS
  LANGUAGE_MODELING
  NER
  SENTIMENT_ANALYSIS
}

enum EndangermentLevel {
  SAFE
  VULNERABLE
  DEFINITELY_ENDANGERED
  SEVERELY_ENDANGERED
  CRITICALLY_ENDANGERED
  EXTINCT
}
```

### GraphQL 쿼리 예제

```graphql
# 언어 정보와 지원 모델 조회
query GetLanguageInfo {
  language(code: "ain") {
    code
    name
    nativeName
    family
    endangerment
    speakerCount
    supportedTasks
    models {
      id
      name
      task
      accuracy
      parameters
    }
  }
}

# 번역과 음성 합성을 하나의 요청으로
mutation TranslateAndSpeak {
  translation: translate(
    text: "Hello"
    sourceLang: "en"
    targetLang: "ain"
  ) {
    translation
    confidence
  }

  speech: synthesizeSpeech(
    text: $translation.translation
    language: "ain"
    voice: "ain-female-01"
  ) {
    audio
    duration
  }
}

# 여러 언어의 위기 상태 조회
query EndangeredLanguages {
  languages(
    filter: {
      endangerment: [CRITICALLY_ENDANGERED, SEVERELY_ENDANGERED]
      hasModels: true
    }
  ) {
    code
    name
    endangerment
    speakerCount
    models {
      task
      accuracy
    }
  }
}
```

## 인증 및 권한 관리

### API 키 기반 인증

```http
GET /v1/translate
Authorization: Bearer wia_sk_abc123xyz789
```

**API 키 생성**:
```json
POST /v1/auth/keys

{
  "name": "My Ainu Translation App",
  "permissions": ["translate", "asr", "tts"],
  "rate_limit": {
    "requests_per_minute": 60,
    "requests_per_day": 10000
  },
  "allowed_languages": ["ain", "ja", "en"]
}

// 응답
{
  "status": "success",
  "data": {
    "key": "wia_sk_abc123xyz789",
    "key_id": "key_001",
    "created_at": "2025-03-15T10:00:00Z",
    "expires_at": null
  }
}
```

### OAuth 2.0 흐름

```
사용자 인증 흐름
┌──────────┐                  ┌──────────┐                 ┌──────────┐
│  Client  │                  │   WIA    │                 │   User   │
│   App    │                  │   Auth   │                 │          │
└────┬─────┘                  └────┬─────┘                 └────┬─────┘
     │                             │                            │
     │  1. Authorization Request   │                            │
     ├─────────────────────────────>                            │
     │                             │                            │
     │  2. Redirect to Login       │                            │
     │<─────────────────────────────┤                            │
     │                             │  3. Login & Consent        │
     │                             ├────────────────────────────>│
     │                             │                            │
     │                             │  4. Authorization Code     │
     │  5. Redirect with Code      │<────────────────────────────┤
     │<─────────────────────────────┤                            │
     │                             │                            │
     │  6. Exchange Code for Token │                            │
     ├─────────────────────────────>                            │
     │                             │                            │
     │  7. Access Token + Refresh  │                            │
     │<─────────────────────────────┤                            │
     │                             │                            │
```

**토큰 교환**:
```http
POST /v1/auth/token
Content-Type: application/x-www-form-urlencoded

grant_type=authorization_code&
code=AUTH_CODE&
client_id=CLIENT_ID&
client_secret=CLIENT_SECRET&
redirect_uri=REDIRECT_URI
```

### 권한 레벨

| 권한 | 설명 | 허용 작업 |
|-----|------|----------|
| `read` | 읽기 전용 | 모델 정보 조회, 언어 정보 조회 |
| `translate` | 번역 | 번역 API 사용 |
| `asr` | 음성 인식 | ASR API 사용 |
| `tts` | 음성 합성 | TTS API 사용 |
| `train` | 모델 훈련 | 커스텀 모델 미세 조정 |
| `admin` | 관리자 | 모든 작업 + 사용자 관리 |

## SDK 및 클라이언트 라이브러리

### Python SDK

```python
from wia_lang_005 import Client

# 초기화
client = Client(api_key="wia_sk_abc123xyz789")

# 번역
result = client.translate(
    text="イランカラプテ",
    source_lang="ain",
    target_lang="ja"
)
print(result.translation)  # "こんにちは"
print(result.confidence)   # 0.92

# 음성 인식
with open("audio.wav", "rb") as audio_file:
    transcription = client.transcribe(
        audio=audio_file,
        language="ain"
    )
    print(transcription.text)

# 음성 합성
audio = client.synthesize(
    text="イランカラプテ",
    language="ain",
    voice="ain-female-01"
)
audio.save("output.mp3")

# 배치 처리
translations = client.translate_batch(
    texts=["Hello", "Goodbye", "Thank you"],
    source_lang="en",
    target_lang="ain"
)
for t in translations:
    print(f"{t.source} -> {t.translation}")

# 비동기 처리
import asyncio

async def translate_async():
    result = await client.translate_async(
        text="Hello",
        source_lang="en",
        target_lang="ain"
    )
    return result

asyncio.run(translate_async())
```

### JavaScript/TypeScript SDK

```typescript
import { WIALang005Client } from '@wia/lang-005-sdk';

// 초기화
const client = new WIALang005Client({
  apiKey: 'wia_sk_abc123xyz789',
  baseURL: 'https://api.wia-lang-005.org/v1'
});

// 번역
const translation = await client.translate({
  text: 'イランカラプテ',
  sourceLang: 'ain',
  targetLang: 'ja'
});
console.log(translation.data.translation);  // "こんにちは"

// 음성 인식 (브라우저)
const audioBlob = await navigator.mediaDevices.getUserMedia({audio: true});
const transcription = await client.transcribe({
  audio: audioBlob,
  language: 'ain'
});

// 실시간 스트리밍
const stream = client.createASRStream({
  language: 'ain',
  onPartial: (text) => console.log('Partial:', text),
  onFinal: (text) => console.log('Final:', text),
  onError: (error) => console.error(error)
});

// 오디오 데이터 전송
mediaStream.addEventListener('data', (chunk) => {
  stream.send(chunk);
});

// 타입 안정성
interface TranslateOptions {
  text: string;
  sourceLang: string;
  targetLang: string;
  formality?: 'casual' | 'polite' | 'formal';
  domain?: string;
}

const options: TranslateOptions = {
  text: 'Hello',
  sourceLang: 'en',
  targetLang: 'ain',
  formality: 'polite'
};
```

### Java SDK

```java
import org.wia.lang005.WIALang005Client;
import org.wia.lang005.models.*;

public class AinuTranslator {
    public static void main(String[] args) {
        // 클라이언트 초기화
        WIALang005Client client = new WIALang005Client.Builder()
            .apiKey("wia_sk_abc123xyz789")
            .build();

        // 번역
        TranslateRequest request = TranslateRequest.builder()
            .text("イランカラプテ")
            .sourceLang("ain")
            .targetLang("ja")
            .build();

        TranslationResult result = client.translate(request);
        System.out.println(result.getTranslation());  // "こんにちは"
        System.out.println(result.getConfidence());   // 0.92

        // 음성 인식
        File audioFile = new File("recording.wav");
        ASRRequest asrRequest = ASRRequest.builder()
            .audio(audioFile)
            .language("ain")
            .build();

        ASRResult transcription = client.transcribe(asrRequest);
        System.out.println(transcription.getText());

        // 비동기 처리
        CompletableFuture<TranslationResult> future =
            client.translateAsync(request);

        future.thenAccept(res -> {
            System.out.println(res.getTranslation());
        });
    }
}
```

## 오류 처리

### 표준 오류 응답

```json
{
  "status": "error",
  "error": {
    "code": "invalid_language",
    "message": "Language code 'xyz' is not supported",
    "details": {
      "supported_languages": ["ain", "chr", "nav", "haw"],
      "requested": "xyz"
    },
    "request_id": "req_error_123",
    "timestamp": "2025-03-15T10:30:00Z",
    "documentation_url": "https://docs.wia-lang-005.org/errors/invalid_language"
  }
}
```

### 오류 코드

| 코드 | HTTP Status | 설명 | 해결 방법 |
|-----|------------|------|----------|
| `invalid_api_key` | 401 | API 키 오류 | 유효한 API 키 사용 |
| `rate_limit_exceeded` | 429 | 속도 제한 초과 | 요청 속도 줄이기 |
| `invalid_language` | 400 | 지원하지 않는 언어 | 지원 언어 목록 확인 |
| `audio_too_large` | 413 | 오디오 파일 너무 큼 | 25MB 이하로 압축 |
| `model_not_found` | 404 | 모델 없음 | 사용 가능 모델 조회 |
| `insufficient_credits` | 402 | 크레딧 부족 | 크레딧 충전 |
| `server_error` | 500 | 서버 오류 | 재시도 또는 지원팀 연락 |

### SDK 오류 처리 예제

```python
from wia_lang_005 import Client, WIAError, RateLimitError, InvalidLanguageError

client = Client(api_key="wia_sk_abc123xyz789")

try:
    result = client.translate(
        text="Hello",
        source_lang="en",
        target_lang="xyz"  # 존재하지 않는 언어
    )
except InvalidLanguageError as e:
    print(f"Invalid language: {e.details['requested']}")
    print(f"Supported: {e.details['supported_languages']}")
except RateLimitError as e:
    print(f"Rate limit exceeded. Retry after {e.retry_after} seconds")
    time.sleep(e.retry_after)
    # 재시도
except WIAError as e:
    print(f"API Error: {e.code} - {e.message}")
    print(f"Request ID: {e.request_id}")
```

## 속도 제한 (Rate Limiting)

### 제한 정책

| 티어 | 분당 요청 | 일일 요청 | 월별 크레딧 | 비용 |
|-----|---------|---------|-----------|------|
| Free | 20 | 1,000 | 10,000 | $0 |
| Researcher | 60 | 10,000 | 100,000 | $29/월 |
| Professional | 300 | 100,000 | 1,000,000 | $99/월 |
| Enterprise | 무제한 | 무제한 | Custom | 문의 |

### 응답 헤더

```http
HTTP/1.1 200 OK
X-RateLimit-Limit: 60
X-RateLimit-Remaining: 45
X-RateLimit-Reset: 1710504000
Retry-After: 30
```

---

## 핵심 요약

**주요 내용:**

1. **RESTful API**: 번역, ASR, TTS, 언어 모델링, NER 등 주요 작업을 위한 통일된 REST API를 제공합니다.

2. **GraphQL 지원**: 유연한 쿼리와 여러 작업을 하나의 요청으로 처리할 수 있는 GraphQL API를 제공합니다.

3. **인증 및 권한**: API 키와 OAuth 2.0을 통한 안전한 인증과 세분화된 권한 관리를 지원합니다.

4. **다양한 SDK**: Python, JavaScript/TypeScript, Java 등 주요 프로그래밍 언어를 위한 공식 SDK를 제공합니다.

5. **오류 처리**: 표준화된 오류 응답과 명확한 오류 코드로 디버깅을 용이하게 합니다.

6. **속도 제한**: 공정한 사용을 위한 계층별 속도 제한 정책을 시행합니다.

---

## 복습 문제

1. RESTful API와 GraphQL API의 주요 차이점을 설명하고, 각각이 유리한 사용 사례를 제시하시오.

2. WIA-LANG-005 API의 일반 요청 및 응답 형식을 설명하고, 메타데이터에 포함되어야 하는 필수 정보를 나열하시오.

3. OAuth 2.0 인증 흐름의 각 단계를 설명하고, API 키 기반 인증과 비교하여 장단점을 논하시오.

4. 음성 인식 API에서 배치 처리와 실시간 스트리밍 방식의 차이를 설명하고, 각각의 적합한 사용 사례를 제시하시오.

5. SDK를 사용할 때 오류 처리의 중요성을 설명하고, 재시도 로직을 구현해야 하는 오류 유형을 나열하시오.

6. 속도 제한(Rate Limiting)이 필요한 이유를 설명하고, 클라이언트가 속도 제한에 도달했을 때 적절히 처리하는 방법을 서술하시오.

---

## 다음 장 미리보기

제6장에서는 Phase 3: 프로토콜을 상세히 다룹니다. gRPC 통신, Protocol Buffers 정의, 모델 배포 프로토콜, 보안 및 암호화, 그리고 실시간 스트리밍 프로토콜을 살펴봅니다.

---

**제5장 완료** | 예상 페이지: 21

[이전: 제4장 - 데이터 형식](./04-데이터-형식.md) | [다음: 제6장 - 프로토콜](./06-프로토콜.md)

---

© 2025 SmileStory Inc. / WIA
弘益人間 (홍익인간) · Benefit All Humanity
