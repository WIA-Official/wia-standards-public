# 5장: API 인터페이스

## RESTful API 설계

WIA-LANG-007 표준은 수어 시스템 통합을 위한 포괄적인 API 인터페이스를 정의합니다. RESTful API는 기본 인터페이스로, 간단하고 널리 지원되며 캐싱과 확장이 용이합니다.

### API 기본 구조

#### 베이스 URL 및 버전 관리

```typescript
interface APIConfiguration {
  production: {
    base_url: "https://api.wia-lang-007.org",
    version: "/v1",
    full_endpoint: "https://api.wia-lang-007.org/v1"
  },

  regional_endpoints: {
    korea: "https://kr.api.wia-lang-007.org/v1",
    us: "https://us.api.wia-lang-007.org/v1",
    eu: "https://eu.api.wia-lang-007.org/v1",
    global: "https://api.wia-lang-007.org/v1"  // CDN 자동 라우팅
  },

  versioning: {
    strategy: "URL path versioning",
    current: "v1",
    deprecated: [],
    beta: "v2-beta (2026 Q1 예정)"
  }
}
```

#### 인증 및 권한

```typescript
interface Authentication {
  methods: {
    api_key: {
      header: "X-WIA-API-Key",
      format: "wia_live_abc123...",
      use_case: "서버 간 통신",
      rate_limit: "10,000 requests/hour"
    },

    oauth2: {
      grant_types: ["authorization_code", "client_credentials"],
      scopes: [
        "signs:read",
        "signs:write",
        "videos:upload",
        "avatar:generate",
        "translate:realtime"
      ],
      token_endpoint: "/oauth/token",
      authorize_endpoint: "/oauth/authorize"
    },

    jwt: {
      algorithm: "RS256",
      expiry: "1 hour",
      refresh: "7 days",
      claims: {
        sub: "user_id",
        iss: "wia-lang-007.org",
        aud: "api.wia-lang-007.org",
        scope: "signs:read videos:upload"
      }
    }
  },

  rate_limiting: {
    free_tier: {
      requests_per_hour: 1000,
      requests_per_day: 10000,
      concurrent_requests: 10
    },
    pro_tier: {
      requests_per_hour: 10000,
      requests_per_day: 100000,
      concurrent_requests: 50
    },
    enterprise: {
      requests_per_hour: "unlimited",
      requests_per_day: "unlimited",
      concurrent_requests: 500
    }
  }
}
```

**인증 예시:**

```bash
# API 키 인증
curl -H "X-WIA-API-Key: wia_live_abc123..." \
  https://api.wia-lang-007.org/v1/signs/ksl/hello

# OAuth 2.0 Bearer 토큰
curl -H "Authorization: Bearer eyJhbGciOiJSUzI1NiIs..." \
  https://api.wia-lang-007.org/v1/signs/ksl/hello
```

### 핵심 API 엔드포인트

#### 수어 사전 API

```typescript
interface SignDictionaryAPI {
  // 1. 수어 검색
  GET_search: {
    endpoint: "/signs/search",
    method: "GET",
    query_params: {
      q: "검색어",
      language: "KSL | ASL | BSL | ...",
      category: "인사 | 숫자 | 색상 | ...",
      difficulty: "초급 | 중급 | 고급",
      limit: 20,
      offset: 0
    },
    example: `
GET /v1/signs/search?q=안녕&language=KSL&limit=10

Response:
{
  "results": [
    {
      "id": "ksl_hello_formal",
      "gloss": "HELLO",
      "ksl": "안녕하세요",
      "video_url": "https://cdn.wia.org/ksl/hello.mp4",
      "thumbnail": "https://cdn.wia.org/ksl/hello_thumb.jpg",
      "duration": 1.2,
      "category": "인사",
      "difficulty": "초급",
      "frequency": "very_common"
    }
  ],
  "total": 23,
  "page": 1,
  "per_page": 10
}
    `
  },

  // 2. 특정 수어 조회
  GET_sign: {
    endpoint: "/signs/{language}/{sign_id}",
    method: "GET",
    path_params: {
      language: "KSL",
      sign_id: "hello_formal"
    },
    example: `
GET /v1/signs/ksl/hello_formal

Response:
{
  "id": "ksl_hello_formal",
  "gloss": "HELLO_FORMAL",
  "translations": {
    "ko": "안녕하세요",
    "en": "Hello (formal)",
    "ja": "こんにちは"
  },
  "video": {
    "hd": "https://cdn.wia.org/ksl/hello_1080p60.mp4",
    "4k": "https://cdn.wia.org/ksl/hello_4k60.mp4",
    "thumbnail": "https://cdn.wia.org/ksl/hello_thumb.jpg"
  },
  "notation": {
    "hamnosys": "ㅎ︎🤚︎⬇︎",
    "signwriting": "✋⬇️👤",
    "gloss": "HELLO_FORMAL"
  },
  "metadata": {
    "handshape": "flat_hand",
    "location": "chest",
    "movement": "forward_bow",
    "duration": 1.2,
    "difficulty": "초급",
    "frequency": "very_common"
  },
  "related_signs": [
    "hello_casual",
    "goodbye",
    "nice_to_meet"
  ]
}
    `
  },

  // 3. 수어 업로드
  POST_sign: {
    endpoint: "/signs/{language}",
    method: "POST",
    headers: {
      "Content-Type": "multipart/form-data",
      "Authorization": "Bearer {token}"
    },
    body: {
      video: "File",
      gloss: "THANK_YOU",
      translation: "감사합니다",
      category: "인사",
      notation: {
        hamnosys: "...",
        signwriting: "..."
      }
    }
  }
}
```

#### 비디오 처리 API

```typescript
interface VideoProcessingAPI {
  // 1. 비디오 업로드
  POST_upload: {
    endpoint: "/videos/upload",
    method: "POST",
    headers: {
      "Content-Type": "multipart/form-data"
    },
    body: {
      file: "video.mp4",
      language: "KSL",
      metadata: {
        signer_id: "signer_001",
        recording_date: "2025-12-29",
        location: "Seoul"
      }
    },
    response: {
      video_id: "vid_abc123",
      status: "processing",
      estimated_time: 120  // 초
    }
  },

  // 2. 비디오 트랜스코딩
  POST_transcode: {
    endpoint: "/videos/{video_id}/transcode",
    method: "POST",
    body: {
      formats: ["1080p60", "720p60", "480p30"],
      codecs: ["H.265", "VP9"],
      optimize_for: "web"
    },
    webhook: "https://your-app.com/webhook/transcode"
  },

  // 3. 비디오 분석
  POST_analyze: {
    endpoint: "/videos/{video_id}/analyze",
    method: "POST",
    body: {
      features: [
        "pose_estimation",
        "hand_tracking",
        "facial_landmarks",
        "sign_recognition"
      ]
    },
    response: {
      analysis_id: "ana_xyz789",
      status: "processing",
      estimated_time: 60
    }
  },

  // 4. 분석 결과 조회
  GET_analysis: {
    endpoint: "/videos/{video_id}/analysis/{analysis_id}",
    method: "GET",
    response: `
{
  "video_id": "vid_abc123",
  "analysis_id": "ana_xyz789",
  "status": "completed",
  "results": {
    "signs_detected": [
      {
        "gloss": "HELLO",
        "confidence": 0.95,
        "start_time": 0.0,
        "end_time": 1.2,
        "handshape": "flat_hand",
        "location": "chest"
      },
      {
        "gloss": "THANK_YOU",
        "confidence": 0.92,
        "start_time": 1.5,
        "end_time": 2.8
      }
    ],
    "pose_data": {
      "format": "MediaPipe",
      "url": "https://cdn.wia.org/analysis/ana_xyz789_pose.json"
    },
    "quality_metrics": {
      "video_quality": 0.98,
      "lighting": 0.95,
      "visibility": 0.99,
      "motion_blur": 0.02
    }
  }
}
    `
  }
}
```

#### 실시간 번역 API

```typescript
interface TranslationAPI {
  // 1. 수어 -> 텍스트
  POST_sign_to_text: {
    endpoint: "/translate/sign-to-text",
    method: "POST",
    body: {
      video_url: "https://your-app.com/video.mp4",
      source_language: "KSL",
      target_language: "ko",
      mode: "realtime | batch"
    },
    response: {
      translation_id: "trans_123",
      status: "processing | completed",
      result: {
        text: "안녕하세요, 만나서 반갑습니다.",
        confidence: 0.89,
        segments: [
          {
            text: "안녕하세요",
            start: 0.0,
            end: 1.2,
            confidence: 0.95
          },
          {
            text: "만나서 반갑습니다",
            start: 1.5,
            end: 3.2,
            confidence: 0.83
          }
        ]
      }
    }
  },

  // 2. 텍스트 -> 수어
  POST_text_to_sign: {
    endpoint: "/translate/text-to-sign",
    method: "POST",
    body: {
      text: "날씨가 정말 좋네요",
      source_language: "ko",
      target_sign_language: "KSL",
      output_format: "video | avatar | notation"
    },
    response: {
      translation_id: "trans_456",
      avatar_url: "https://cdn.wia.org/avatars/trans_456.mp4",
      notation: {
        gloss: "WEATHER REALLY GOOD",
        hamnosys: "...",
        signwriting: "..."
      },
      estimated_duration: 2.5
    }
  },

  // 3. 수어 간 번역
  POST_sign_to_sign: {
    endpoint: "/translate/sign-to-sign",
    method: "POST",
    body: {
      video_url: "https://your-app.com/ksl_video.mp4",
      source_language: "KSL",
      target_language: "ASL",
      preserve_cultural_context: true
    }
  }
}
```

#### 아바타 생성 API

```typescript
interface AvatarAPI {
  // 1. 아바타 생성
  POST_generate: {
    endpoint: "/avatar/generate",
    method: "POST",
    body: {
      signs: [
        {
          gloss: "HELLO",
          duration: 1.2
        },
        {
          gloss: "NICE_MEET",
          duration: 1.8
        }
      ],
      avatar_config: {
        gender: "female",
        skin_tone: "medium",
        clothing: "casual",
        background: "neutral_gray"
      },
      output: {
        format: "mp4",
        resolution: "1080p60",
        quality: "high"
      }
    },
    response: {
      avatar_id: "ava_789",
      status: "rendering",
      estimated_time: 30,
      webhook_url: "optional"
    }
  },

  // 2. 아바타 커스터마이징
  POST_customize: {
    endpoint: "/avatar/customize",
    method: "POST",
    body: {
      base_avatar: "default_female",
      customizations: {
        hair_color: "#2C1B18",
        eye_color: "#4A3728",
        clothing: {
          top: "tshirt_blue",
          bottom: "jeans"
        },
        accessories: ["glasses"]
      }
    },
    response: {
      custom_avatar_id: "custom_ava_123",
      preview_url: "https://cdn.wia.org/avatars/preview_123.jpg"
    }
  },

  // 3. 실시간 아바타 스트리밍
  WebSocket_stream: {
    endpoint: "wss://api.wia-lang-007.org/v1/avatar/stream",
    protocol: "WebSocket",
    usage: `
// 클라이언트 연결
const ws = new WebSocket('wss://api.wia-lang-007.org/v1/avatar/stream');

// 텍스트 전송
ws.send(JSON.stringify({
  type: 'text_to_sign',
  text: '안녕하세요',
  language: 'KSL'
}));

// 아바타 애니메이션 수신
ws.onmessage = (event) => {
  const frame = JSON.parse(event.data);
  // frame.boneRotations = {...}
  // 아바타 렌더링 업데이트
};
    `
  }
}
```

### GraphQL API

GraphQL은 복잡한 쿼리와 정확한 데이터 요청에 유용합니다.

```graphql
# 스키마 정의
type Sign {
  id: ID!
  gloss: String!
  translations: [Translation!]!
  video: Video!
  notation: Notation!
  metadata: SignMetadata!
  relatedSigns: [Sign!]!
}

type Video {
  id: ID!
  url: String!
  thumbnail: String!
  duration: Float!
  quality: VideoQuality!
  formats: [VideoFormat!]!
}

type VideoQuality {
  resolution: String!
  frameRate: Int!
  bitrate: Int!
  codec: String!
}

type Translation {
  language: String!
  text: String!
}

type Notation {
  hamnosys: String
  signwriting: String
  gloss: String!
}

type SignMetadata {
  handshape: String
  location: String
  movement: String
  difficulty: Difficulty!
  frequency: Frequency!
  category: String!
}

enum Difficulty {
  BEGINNER
  INTERMEDIATE
  ADVANCED
}

enum Frequency {
  VERY_COMMON
  COMMON
  UNCOMMON
  RARE
}

# 쿼리
type Query {
  # 단일 수어 조회
  sign(language: String!, id: ID!): Sign

  # 수어 검색
  searchSigns(
    query: String!
    language: String!
    category: String
    difficulty: Difficulty
    limit: Int = 20
    offset: Int = 0
  ): SignSearchResult!

  # 비디오 분석
  analyzeVideo(videoId: ID!): VideoAnalysis

  # 번역
  translateSignToText(
    videoUrl: String!
    sourceLanguage: String!
    targetLanguage: String!
  ): TranslationResult!
}

type Mutation {
  # 수어 업로드
  uploadSign(input: UploadSignInput!): Sign!

  # 아바타 생성
  generateAvatar(input: GenerateAvatarInput!): Avatar!

  # 비디오 트랜스코드
  transcodeVideo(videoId: ID!, formats: [String!]!): TranscodeJob!
}

# 구독 (실시간)
type Subscription {
  # 번역 진행 상황
  translationProgress(translationId: ID!): TranslationProgress!

  # 아바타 렌더링 진행
  avatarRenderProgress(avatarId: ID!): RenderProgress!
}
```

**GraphQL 쿼리 예시:**

```graphql
# 한국수어 인사 수어 검색
query SearchKSLGreetings {
  searchSigns(
    query: "인사"
    language: "KSL"
    category: "인사"
    difficulty: BEGINNER
    limit: 10
  ) {
    results {
      id
      gloss
      translations {
        language
        text
      }
      video {
        url
        thumbnail
        duration
        quality {
          resolution
          frameRate
        }
      }
      notation {
        gloss
        hamnosys
      }
      metadata {
        difficulty
        frequency
        category
      }
    }
    total
  }
}

# 응답
{
  "data": {
    "searchSigns": {
      "results": [
        {
          "id": "ksl_hello_formal",
          "gloss": "HELLO_FORMAL",
          "translations": [
            {"language": "ko", "text": "안녕하세요"},
            {"language": "en", "text": "Hello (formal)"}
          ],
          "video": {
            "url": "https://cdn.wia.org/ksl/hello_1080p60.mp4",
            "thumbnail": "https://cdn.wia.org/ksl/hello_thumb.jpg",
            "duration": 1.2,
            "quality": {
              "resolution": "1080p",
              "frameRate": 60
            }
          },
          "notation": {
            "gloss": "HELLO_FORMAL",
            "hamnosys": "ㅎ︎🤚︎⬇︎"
          },
          "metadata": {
            "difficulty": "BEGINNER",
            "frequency": "VERY_COMMON",
            "category": "인사"
          }
        }
      ],
      "total": 23
    }
  }
}
```

### WebSocket 실시간 API

실시간 수어 인식 및 번역을 위한 WebSocket API:

```typescript
interface WebSocketAPI {
  connection: {
    url: "wss://api.wia-lang-007.org/v1/realtime",
    protocols: ["wia-sign-v1"],
    auth: "?token=your_jwt_token"
  },

  message_types: {
    // 1. 비디오 스트림 전송
    video_frame: {
      type: "video_frame",
      data: "base64_encoded_frame",
      timestamp: 1234567890,
      sequence: 42
    },

    // 2. 수어 인식 결과 수신
    recognition_result: {
      type: "recognition",
      signs: [
        {
          gloss: "HELLO",
          confidence: 0.95,
          start_time: 0.0,
          end_time: 1.2
        }
      ],
      text: "안녕하세요",
      timestamp: 1234567890
    },

    // 3. 아바타 애니메이션 스트리밍
    avatar_frame: {
      type: "avatar_animation",
      boneRotations: {
        rightHand: {wrist: [0, 0, 0], thumb: [...], ...},
        leftHand: {...},
        spine: {...},
        head: {...}
      },
      facialExpressions: {
        eyebrowsRaised: 0.8,
        smile: 0.3
      },
      timestamp: 1234567890
    }
  }
}
```

**WebSocket 사용 예시 (JavaScript):**

```javascript
// 연결 설정
const ws = new WebSocket(
  'wss://api.wia-lang-007.org/v1/realtime?token=' + apiToken
);

// 연결 성공
ws.onopen = () => {
  console.log('WebSocket 연결됨');

  // 설정 전송
  ws.send(JSON.stringify({
    type: 'configure',
    language: 'KSL',
    mode: 'sign_to_text',
    options: {
      continuous: true,
      interim_results: true
    }
  }));
};

// 비디오 프레임 전송 (60fps)
function sendVideoFrame(videoElement) {
  const canvas = document.createElement('canvas');
  canvas.width = videoElement.videoWidth;
  canvas.height = videoElement.videoHeight;

  const ctx = canvas.getContext('2d');
  ctx.drawImage(videoElement, 0, 0);

  const frameData = canvas.toDataURL('image/jpeg', 0.8)
    .split(',')[1];  // base64

  ws.send(JSON.stringify({
    type: 'video_frame',
    data: frameData,
    timestamp: Date.now()
  }));
}

// 60fps로 프레임 전송
setInterval(() => {
  sendVideoFrame(videoElement);
}, 1000 / 60);

// 인식 결과 수신
ws.onmessage = (event) => {
  const message = JSON.parse(event.data);

  switch(message.type) {
    case 'recognition':
      console.log('인식된 수어:', message.text);
      updateSubtitles(message.text);
      break;

    case 'interim':
      console.log('중간 결과:', message.text);
      updateInterimText(message.text);
      break;

    case 'error':
      console.error('오류:', message.error);
      break;
  }
};

// 연결 종료
ws.onclose = () => {
  console.log('WebSocket 연결 종료');
};
```

### SDK 및 클라이언트 라이브러리

```typescript
// TypeScript/JavaScript SDK
import { WIALang007 } from '@wia/lang-007-sdk';

const client = new WIALang007({
  apiKey: process.env.WIA_API_KEY,
  region: 'kr'  // 한국 리전
});

// 수어 검색
const signs = await client.signs.search({
  query: '안녕',
  language: 'KSL',
  limit: 10
});

// 비디오 업로드 및 분석
const video = await client.videos.upload({
  file: videoFile,
  language: 'KSL'
});

const analysis = await client.videos.analyze(video.id, {
  features: ['sign_recognition', 'pose_estimation']
});

// 실시간 번역
const stream = client.realtime.createStream({
  mode: 'sign_to_text',
  language: 'KSL'
});

stream.on('recognition', (result) => {
  console.log('인식:', result.text);
});

stream.sendFrame(videoFrame);

// 아바타 생성
const avatar = await client.avatar.generate({
  text: '안녕하세요, 반갑습니다',
  language: 'KSL',
  avatarConfig: {
    gender: 'female',
    style: 'realistic'
  }
});
```

```python
# Python SDK
from wia_lang_007 import Client

client = Client(api_key=os.getenv('WIA_API_KEY'), region='kr')

# 수어 검색
signs = client.signs.search(
    query='안녕',
    language='KSL',
    limit=10
)

# 비디오 분석
with open('sign_video.mp4', 'rb') as f:
    video = client.videos.upload(f, language='KSL')

analysis = client.videos.analyze(
    video.id,
    features=['sign_recognition', 'pose_estimation']
)

# 실시간 번역
for frame in video_stream:
    result = client.realtime.recognize(frame, language='KSL')
    print(f"인식: {result.text}")
```

### 오류 처리

```typescript
interface ErrorResponse {
  error: {
    code: string,
    message: string,
    details?: object,
    timestamp: string,
    request_id: string
  }
}

// 표준 오류 코드
enum ErrorCode {
  // 4xx 클라이언트 오류
  INVALID_REQUEST = "INVALID_REQUEST",
  AUTHENTICATION_FAILED = "AUTHENTICATION_FAILED",
  PERMISSION_DENIED = "PERMISSION_DENIED",
  RESOURCE_NOT_FOUND = "RESOURCE_NOT_FOUND",
  RATE_LIMIT_EXCEEDED = "RATE_LIMIT_EXCEEDED",
  INVALID_VIDEO_FORMAT = "INVALID_VIDEO_FORMAT",
  SIGN_NOT_RECOGNIZED = "SIGN_NOT_RECOGNIZED",

  // 5xx 서버 오류
  INTERNAL_ERROR = "INTERNAL_ERROR",
  SERVICE_UNAVAILABLE = "SERVICE_UNAVAILABLE",
  PROCESSING_TIMEOUT = "PROCESSING_TIMEOUT"
}
```

### 철학: 弘益人間 (널리 인간을 이롭게 하라)

API는 기술의 다리입니다. 弘益人間 원칙으로:

**개방성:** 모든 개발자가 수어 기술을 구축할 수 있도록.

**접근성:** 간단한 API로 복잡한 수어 AI를 민주화.

**신뢰성:** 안정적인 서비스로 중요한 의사소통 지원.

**혁신:** 개발자들이 새로운 수어 애플리케이션을 만들도록 권한 부여.

API는 수어 기술을 모든 사람에게 가져옵니다.

---

**다음 장:** 프로토콜 - 통신 프로토콜, 보안, 동기화 메커니즘을 탐구합니다.

---

© 2025 SmileStory Inc. / WIA
弘益人間 (홍익인간) · Benefit All Humanity
