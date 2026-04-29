# 5장: API 인터페이스

## 프로그래밍 방식 접근

WIA-LANG-003은 구술 전통 데이터에 프로그래밍 방식으로 접근할 수 있는 포괄적인 API (Application Programming Interface) 사양을 정의합니다. 이를 통해 연구자, 개발자, 기관이 자동화된 도구를 구축하고, 대규모 분석을 수행하며, 다른 시스템과 통합할 수 있습니다.

### API 아키텍처

WIA-LANG-003은 다양한 사용 사례를 지원하는 다층 API 아키텍처를 정의합니다:

```typescript
interface WIA_LANG_003_API_Architecture {
  // 레이어 1: REST API (주 인터페이스)
  restAPI: {
    standard: "OpenAPI 3.0";
    baseURL: "https://api.oral-traditions.org/v1";
    authentication: "OAuth 2.0";
    format: "JSON" | "XML";
    pagination: true;
    rateLimit: "1000 requests/hour";
  };

  // 레이어 2: GraphQL API (유연한 쿼리)
  graphqlAPI: {
    endpoint: "https://api.oral-traditions.org/graphql";
    schema: "WIA-LANG-003 GraphQL Schema";
    playground: true;                 // 인터랙티브 탐색
    subscriptions: true;              // 실시간 업데이트
  };

  // 레이어 3: OAI-PMH (메타데이터 수확)
  oaiPMH: {
    version: "2.0";
    baseURL: "https://api.oral-traditions.org/oai";
    metadataFormats: ["oai_dc", "mods", "olac"];
    sets: true;                       // 컬렉션별 그룹화
  };

  // 레이어 4: IIIF (미디어 전달)
  iiif: {
    imageAPI: "3.0";
    presentationAPI: "3.0";
    baseURL: "https://iiif.oral-traditions.org";
    features: ["region", "size", "rotation", "quality"];
  };

  // 레이어 5: 검색 API
  searchAPI: {
    protocol: "SRU 2.0";              // Search/Retrieve via URL
    endpoint: "https://api.oral-traditions.org/search";
    queryLanguage: "CQL";             // Contextual Query Language
    facets: true;
  };
}
```

### REST API 상세 사양

#### 1. 인증 및 권한 부여

```typescript
interface Authentication {
  // OAuth 2.0 흐름
  oauth2: {
    authorizationURL: "https://api.oral-traditions.org/oauth/authorize";
    tokenURL: "https://api.oral-traditions.org/oauth/token";
    grantTypes: ["authorization_code", "client_credentials", "refresh_token"];
    scopes: {
      "read:public": "공개 데이터 읽기",
      "read:restricted": "제한된 데이터 읽기 (승인 필요)",
      "write:records": "레코드 생성/수정",
      "delete:records": "레코드 삭제",
      "admin": "관리 권한"
    };
  };

  // API 키 (간단한 접근)
  apiKey: {
    header: "X-API-Key";
    registration: "https://api.oral-traditions.org/register";
    rateLimit: "낮음 (100 req/hour)";
  };

  // 문화적 접근 제어
  culturalAccess: {
    community: {
      verification: "커뮤니티 구성원 인증";
      scope: "read:community_restricted";
    };
    elders: {
      verification: "나이 및 커뮤니티 확인";
      scope: "read:sacred";
    };
    gender: {
      verification: "성별 제한 컨텐츠";
      scope: "read:gender_restricted";
    };
  };
}

// 인증 예제
const authExample = {
  // 1. OAuth 토큰 요청
  request: `POST /oauth/token
Content-Type: application/x-www-form-urlencoded

grant_type=client_credentials
&client_id=YOUR_CLIENT_ID
&client_secret=YOUR_CLIENT_SECRET
&scope=read:public read:restricted`,

  // 2. 응답
  response: {
    access_token: "eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...",
    token_type: "Bearer",
    expires_in: 3600,
    scope: "read:public read:restricted"
  },

  // 3. API 요청
  apiRequest: `GET /records/WIA-LANG-003-KOR-001234
Authorization: Bearer eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...
Accept: application/json`
};
```

#### 2. 레코드 엔드포인트

```typescript
interface RecordEndpoints {
  // 레코드 목록 조회
  listRecords: {
    method: "GET";
    path: "/records";

    queryParameters: {
      page: "number (기본: 1)",
      limit: "number (기본: 20, 최대: 100)",
      sort: "field:asc|desc",
      filter: "필드별 필터링",
      language: "ISO 639-3 코드",
      type: "OralTraditionType",
      dateFrom: "ISO 8601 날짜",
      dateTo: "ISO 8601 날짜"
    };

    example: `GET /records?language=kor&type=folktale&limit=50&sort=date:desc`;

    response: {
      metadata: {
        total: 1234,
        page: 1,
        limit: 50,
        pages: 25
      },
      records: [
        {
          recordID: "WIA-LANG-003-KOR-001234",
          title: "해와 달이 된 오누이",
          type: "folktale",
          language: { iso639_3: "kor", name: "한국어" },
          summary: "호랑이를 피해 하늘로 올라간 남매 이야기",
          duration: "PT12M34S",
          recordingDate: "2024-05-10",
          _links: {
            self: "/records/WIA-LANG-003-KOR-001234",
            audio: "/records/WIA-LANG-003-KOR-001234/audio",
            transcript: "/records/WIA-LANG-003-KOR-001234/transcript"
          }
        }
        // ... 더 많은 레코드
      ]
    };
  };

  // 단일 레코드 조회
  getRecord: {
    method: "GET";
    path: "/records/{recordID}";

    pathParameters: {
      recordID: "고유 레코드 식별자"
    };

    queryParameters: {
      include: "performers,recording,transcription,translation,culturalContext"
    };

    example: `GET /records/WIA-LANG-003-KOR-001234?include=performers,transcription`;

    response: {
      recordID: "WIA-LANG-003-KOR-001234",
      title: "해와 달이 된 오누이",
      type: "folktale",
      language: {
        iso639_3: "kor",
        name: "한국어",
        endangerment: "safe",
        speakers: 77000000
      },

      performers: [
        {
          performerID: "PERF-001",
          name: "김영희",
          role: "narrator",
          age: 75,
          community: "강원도",
          biography: "평생 민담을 수집하고 전승해온 구연가"
        }
      ],

      recording: {
        date: "2024-05-10T14:30:00Z",
        location: {
          latitude: 37.8813,
          longitude: 127.7297,
          placeName: "강릉시 문화원"
        },
        duration: "PT12M34S",
        equipment: {
          audioRecorder: "Zoom H6",
          microphone: "Shure SM58",
          videoCamera: "Sony A7 IV"
        },
        format: {
          archival: {
            audio: "WAV/PCM 96kHz 24-bit",
            video: "FFV1/MKV 1080p30"
          },
          access: {
            audio: "AAC 256kbps",
            video: "H.264/MP4 1080p"
          }
        }
      },

      content: {
        summary: "가난한 오누이가 호랑이를 피해 하늘로 올라가 해와 달이 되는 한국 전래 동화",
        themes: ["sibling_bond", "divine_transformation", "escape"],
        keywords: ["오누이", "호랑이", "하늘", "해", "달", "동아줄"],
        motifs: ["D1552 (산 이동)", "F51 (하늘로의 사다리)"],
        atu_type: "ATU 123 (늑대와 아이들)"
      },

      transcription: {
        transcriptionID: "TRANS-001",
        language: "kor",
        type: "orthographic",
        segmentCount: 145,
        wordCount: 892,
        methodology: {
          transcriber: "이민수",
          date: "2024-05-15",
          aiAssisted: true,
          humanVerified: true,
          tool: "ELAN 6.7"
        },
        _links: {
          download: {
            txt: "/records/WIA-LANG-003-KOR-001234/transcript.txt",
            vtt: "/records/WIA-LANG-003-KOR-001234/transcript.vtt",
            eaf: "/records/WIA-LANG-003-KOR-001234/transcript.eaf"
          }
        }
      },

      culturalContext: {
        performanceContext: "문화원 행사에서 지역 주민 대상 구연",
        audienceType: "mixed_ages",
        significance: "한국에서 가장 널리 알려진 민담 중 하나",
        variants: [
          "지역마다 오누이가 해/달이 되는 순서가 다름",
          "일부 버전에서는 호랑이가 떡을 먹고 변장함"
        ],
        relatedTraditions: [
          "일본의 태양 여신 아마테라스 신화",
          "중국의 창어 달 신화"
        ]
      },

      rights: {
        license: "CC-BY-NC-SA 4.0",
        copyright: "2024 강릉문화원",
        accessLevel: "public",
        restrictions: null,
        attribution: "김영희 구연, 강릉문화원 제공"
      },

      preservation: {
        archivalCopies: 3,
        locations: ["본원 서버", "국가 백업", "클라우드"],
        lastVerified: "2024-06-01",
        checksumSHA256: "a1b2c3d4e5f6...",
        migrationHistory: []
      },

      _links: {
        self: "/records/WIA-LANG-003-KOR-001234",
        collection: "/collections/korean-folktales",
        audio: {
          archival: "/media/WIA-LANG-003-KOR-001234/audio/archival.wav",
          access: "/media/WIA-LANG-003-KOR-001234/audio/access.m4a",
          streaming: "/media/WIA-LANG-003-KOR-001234/audio/stream.m3u8"
        },
        video: {
          archival: "/media/WIA-LANG-003-KOR-001234/video/archival.mkv",
          access: "/media/WIA-LANG-003-KOR-001234/video/access.mp4",
          streaming: "/media/WIA-LANG-003-KOR-001234/video/stream.m3u8"
        },
        transcript: "/records/WIA-LANG-003-KOR-001234/transcript",
        translation: "/records/WIA-LANG-003-KOR-001234/translation/eng"
      }
    };
  };

  // 레코드 생성
  createRecord: {
    method: "POST";
    path: "/records";
    authentication: "required (scope: write:records)";

    requestBody: {
      // 필수 필드
      title: "string",
      type: "OralTraditionType",
      language: { iso639_3: "string", name: "string" },
      recordingDate: "ISO 8601 date",

      // 선택 필드
      performers: "Performer[]",
      content: "ContentMetadata",
      rights: "RightsManagement",
      // ...
    };

    response: {
      status: 201,
      location: "/records/WIA-LANG-003-KOR-NEW-ID",
      body: {
        recordID: "WIA-LANG-003-KOR-NEW-ID",
        message: "레코드가 성공적으로 생성되었습니다",
        _links: { self: "/records/WIA-LANG-003-KOR-NEW-ID" }
      }
    };
  };

  // 레코드 업데이트
  updateRecord: {
    method: "PATCH";
    path: "/records/{recordID}";
    authentication: "required (scope: write:records)";

    // JSON Patch (RFC 6902)
    requestBody: [
      {
        op: "replace",
        path: "/content/summary",
        value: "업데이트된 요약"
      },
      {
        op: "add",
        path: "/content/keywords/-",
        value: "새로운_키워드"
      }
    ];

    response: {
      status: 200,
      body: {
        recordID: "WIA-LANG-003-KOR-001234",
        version: "v1.1",
        updated: "2024-06-15T10:30:00Z",
        changes: 2
      }
    };
  };

  // 레코드 삭제
  deleteRecord: {
    method: "DELETE";
    path: "/records/{recordID}";
    authentication: "required (scope: delete:records)";

    response: {
      status: 204,
      body: null
    };
  };
}
```

#### 3. 검색 엔드포인트

```typescript
interface SearchEndpoints {
  // 전체 텍스트 검색
  fullTextSearch: {
    method: "GET";
    path: "/search";

    queryParameters: {
      q: "검색 쿼리",
      fields: "title,summary,transcript (검색할 필드)",
      language: "ISO 639-3 코드",
      type: "OralTraditionType",
      facets: "language,type,performer (패싯 필드)",
      page: "number",
      limit: "number"
    };

    example: `GET /search?q=호랑이&fields=title,transcript&language=kor&facets=type,performer`;

    response: {
      query: "호랑이",
      total: 87,
      page: 1,
      limit: 20,

      // 패싯 (필터링 옵션)
      facets: {
        type: [
          { value: "folktale", count: 45 },
          { value: "legend", count: 23 },
          { value: "myth", count: 12 },
          { value: "song", count: 7 }
        ],
        performer: [
          { value: "김영희", count: 15 },
          { value: "박철수", count: 12 },
          // ...
        ]
      },

      // 결과
      results: [
        {
          recordID: "WIA-LANG-003-KOR-001234",
          title: "해와 달이 된 오누이",
          type: "folktale",
          score: 0.95,                  // 관련성 점수

          // 하이라이트 (매칭 부분)
          highlights: {
            transcript: [
              "...어느 날 <em>호랑이</em>가 나타나서...",
              "...<em>호랑이</em>가 어머니를 잡아먹고..."
            ]
          },

          _links: {
            self: "/records/WIA-LANG-003-KOR-001234"
          }
        }
        // ... 더 많은 결과
      ]
    };
  };

  // 고급 검색 (CQL)
  advancedSearch: {
    method: "POST";
    path: "/search/advanced";

    requestBody: {
      query: {
        bool: {
          must: [
            { match: { type: "folktale" } },
            { range: { recordingDate: { gte: "2020-01-01" } } }
          ],
          should: [
            { match: { "content.keywords": "호랑이" } },
            { match: { "content.keywords": "곰" } }
          ],
          filter: [
            { term: { "language.iso639_3": "kor" } }
          ]
        }
      },
      sort: [
        { recordingDate: "desc" },
        { _score: "desc" }
      ],
      page: 1,
      limit: 20
    };

    response: {
      // 검색 결과 구조는 fullTextSearch와 동일
    };
  };

  // 유사 레코드 찾기
  findSimilar: {
    method: "GET";
    path: "/records/{recordID}/similar";

    queryParameters: {
      limit: "number (기본: 10)",
      similarity: "content|metadata|audio (유사도 기준)"
    };

    example: `GET /records/WIA-LANG-003-KOR-001234/similar?similarity=content&limit=5`;

    response: {
      recordID: "WIA-LANG-003-KOR-001234",
      title: "해와 달이 된 오누이",

      similar: [
        {
          recordID: "WIA-LANG-003-KOR-005678",
          title: "호랑이 형님",
          similarity: 0.78,
          reason: "주제 및 모티프 유사 (호랑이, 변신)",
          _links: { self: "/records/WIA-LANG-003-KOR-005678" }
        },
        {
          recordID: "WIA-LANG-003-KOR-009012",
          title: "선녀와 나무꾼",
          similarity: 0.65,
          reason: "구조 유사 (변신, 하늘)",
          _links: { self: "/records/WIA-LANG-003-KOR-009012" }
        }
        // ...
      ]
    };
  };
}
```

#### 4. 미디어 전달 엔드포인트

```typescript
interface MediaEndpoints {
  // 오디오 스트리밍
  audioStreaming: {
    method: "GET";
    path: "/media/{recordID}/audio/stream.{format}";

    formats: {
      m3u8: "HLS 플레이리스트",
      mpd: "DASH manifest"
    };

    example: `GET /media/WIA-LANG-003-KOR-001234/audio/stream.m3u8`;

    response: {
      contentType: "application/vnd.apple.mpegurl",
      body: `#EXTM3U
#EXT-X-VERSION:3
#EXT-X-STREAM-INF:BANDWIDTH=256000
stream_256k.m3u8
#EXT-X-STREAM-INF:BANDWIDTH=128000
stream_128k.m3u8`
    };
  };

  // 비디오 스트리밍
  videoStreaming: {
    method: "GET";
    path: "/media/{recordID}/video/stream.m3u8";

    // 적응형 비트레이트 스트리밍
    response: {
      contentType: "application/vnd.apple.mpegurl",
      body: `#EXTM3U
#EXT-X-VERSION:3
#EXT-X-STREAM-INF:BANDWIDTH=5000000,RESOLUTION=1920x1080
1080p.m3u8
#EXT-X-STREAM-INF:BANDWIDTH=2500000,RESOLUTION=1280x720
720p.m3u8
#EXT-X-STREAM-INF:BANDWIDTH=1000000,RESOLUTION=854x480
480p.m3u8`
    };
  };

  // 오디오 다운로드
  audioDownload: {
    method: "GET";
    path: "/media/{recordID}/audio/{format}";

    formats: ["archival.wav", "archival.flac", "access.m4a", "access.mp3"];

    queryParameters: {
      quality: "archival|access",
      download: "true (다운로드 강제)"
    };

    example: `GET /media/WIA-LANG-003-KOR-001234/audio/access.m4a?download=true`;

    headers: {
      "Content-Type": "audio/mp4",
      "Content-Disposition": "attachment; filename=\"해와_달이_된_오누이.m4a\"",
      "Content-Length": "15728640"
    };
  };

  // 비디오 다운로드
  videoDownload: {
    method: "GET";
    path: "/media/{recordID}/video/{format}";

    formats: ["archival.mkv", "access.mp4"];

    example: `GET /media/WIA-LANG-003-KOR-001234/video/access.mp4`;
  };

  // 전사본 다운로드
  transcriptDownload: {
    method: "GET";
    path: "/records/{recordID}/transcript.{format}";

    formats: ["txt", "vtt", "eaf", "tei"];

    example: `GET /records/WIA-LANG-003-KOR-001234/transcript.vtt`;

    response: {
      contentType: "text/vtt",
      body: `WEBVTT

1
00:00:00.000 --> 00:00:05.500
옛날 옛적에 가난한 오누이가 살았습니다.

2
00:00:05.500 --> 00:00:12.000
어머니는 부잣집에 일을 하러 가셨어요.
...`
    };
  };
}
```

### GraphQL API

보다 유연한 쿼리를 위한 GraphQL API:

```typescript
interface GraphQLAPI {
  endpoint: "https://api.oral-traditions.org/graphql";

  // 스키마
  schema: `
    type Query {
      # 레코드 조회
      record(id: ID!): OralTraditionRecord
      records(
        page: Int = 1
        limit: Int = 20
        language: String
        type: OralTraditionType
        sort: SortInput
      ): RecordConnection

      # 검색
      search(
        query: String!
        fields: [String]
        filters: SearchFilters
        page: Int = 1
        limit: Int = 20
      ): SearchResult

      # 컬렉션
      collection(id: ID!): Collection
      collections: [Collection]

      # 통계
      statistics: Statistics
    }

    type OralTraditionRecord {
      recordID: ID!
      title: String!
      type: OralTraditionType!
      language: Language!
      performers: [Performer]
      recording: Recording
      content: Content
      transcription: Transcription
      translation(language: String): Translation
      culturalContext: CulturalContext
      rights: Rights
      media: Media
      relatedRecords(limit: Int = 5): [OralTraditionRecord]
    }

    type Language {
      iso639_3: String!
      name: String!
      endangerment: EndangermentLevel
      speakers: Int
    }

    type Performer {
      performerID: ID!
      name: String
      anonymousID: String
      role: PerformerRole!
      age: Int
      community: String
      biography: String
    }

    # ... 더 많은 타입
  `;

  // 쿼리 예제
  exampleQuery: `
    query GetKoreanFolktales {
      records(
        language: "kor"
        type: FOLKTALE
        limit: 10
        sort: { field: RECORDING_DATE, order: DESC }
      ) {
        edges {
          node {
            recordID
            title
            performers {
              name
              role
              community
            }
            content {
              summary
              keywords
            }
            media {
              audio {
                access {
                  url
                  format
                  duration
                }
              }
            }
          }
        }
        pageInfo {
          hasNextPage
          endCursor
        }
        totalCount
      }
    }
  `;

  exampleResponse: {
    data: {
      records: {
        edges: [
          {
            node: {
              recordID: "WIA-LANG-003-KOR-001234",
              title: "해와 달이 된 오누이",
              performers: [
                {
                  name: "김영희",
                  role: "NARRATOR",
                  community: "강원도"
                }
              ],
              content: {
                summary: "가난한 오누이가 호랑이를 피해...",
                keywords: ["오누이", "호랑이", "해", "달"]
              },
              media: {
                audio: {
                  access: {
                    url: "/media/WIA-LANG-003-KOR-001234/audio/access.m4a",
                    format: "AAC",
                    duration: "PT12M34S"
                  }
                }
              }
            }
          }
          // ... 더 많은 결과
        ],
        pageInfo: {
          hasNextPage: true,
          endCursor: "cursor123"
        },
        totalCount: 87
      }
    }
  };
}
```

### SDK 및 클라이언트 라이브러리

WIA-LANG-003은 주요 프로그래밍 언어용 공식 SDK를 제공합니다:

```typescript
// TypeScript/JavaScript SDK
import { WIALang003Client } from '@wia/lang-003-sdk';

const client = new WIALang003Client({
  apiKey: 'YOUR_API_KEY',
  baseURL: 'https://api.oral-traditions.org/v1'
});

// 레코드 검색
const records = await client.records.list({
  language: 'kor',
  type: 'folktale',
  limit: 10
});

// 단일 레코드 조회
const record = await client.records.get('WIA-LANG-003-KOR-001234', {
  include: ['performers', 'transcription']
});

// 검색
const searchResults = await client.search({
  query: '호랑이',
  fields: ['title', 'transcript'],
  language: 'kor'
});

// 미디어 스트리밍
const audioStream = await client.media.streamAudio(record.recordID);

// 전사본 다운로드
const transcript = await client.transcripts.download(
  record.recordID,
  'vtt'
);
```

```python
# Python SDK
from wia_lang_003 import Client

client = Client(api_key='YOUR_API_KEY')

# 레코드 검색
records = client.records.list(
    language='kor',
    type='folktale',
    limit=10
)

# 단일 레코드 조회
record = client.records.get(
    'WIA-LANG-003-KOR-001234',
    include=['performers', 'transcription']
)

# 검색
results = client.search(
    query='호랑이',
    fields=['title', 'transcript'],
    language='kor'
)

# 미디어 다운로드
client.media.download_audio(
    record.record_id,
    format='m4a',
    output_path='audio.m4a'
)

# 전사본 다운로드
transcript = client.transcripts.download(
    record.record_id,
    format='vtt'
)
```

### 철학: 개방형 접근

WIA-LANG-003 API는 弘익人間 철학을 구현합니다:

**민주적 접근:** 표준화된 API를 통해 누구나 (연구자, 학생, 개발자) 구술 전통 데이터에 접근하고 활용할 수 있습니다.

**상호운용성:** REST, GraphQL, OAI-PMH 등 다양한 프로토콜 지원으로 기존 시스템과 원활하게 통합됩니다.

**투명성:** OpenAPI 사양과 공개 문서화로 API 동작이 완전히 투명하고 이해 가능합니다.

**문화적 존중:** 접근 제어와 권한 관리를 통해 문화적 프로토콜과 공동체의 권리를 존중합니다.

---

## 장 요약

이 장에서는 WIA-LANG-003 API 인터페이스를 살펴보았습니다:
- 다층 API 아키텍처 (REST, GraphQL, OAI-PMH, IIIF, 검색)
- OAuth 2.0 인증 및 문화적 접근 제어
- 레코드, 검색, 미디어 엔드포인트
- GraphQL API의 유연한 쿼리
- 공식 SDK (TypeScript, Python)
- 개방형 접근과 문화적 존중의 균형

## 복습 질문

1. WIA-LANG-003의 5개 API 레이어는 무엇인가?
2. OAuth 2.0의 주요 스코프는?
3. REST API에서 레코드를 검색하는 방법은?
4. GraphQL의 장점은 무엇인가?
5. 적응형 스트리밍 (HLS/DASH)이란?
6. 문화적 접근 제어가 중요한 이유는?
7. SDK가 API 사용을 어떻게 단순화하는가?
8. API가 弘익人間 철학을 어떻게 구현하는가?

---

**다음 장:** WIA-LANG-003 프로토콜을 다루며, 기록, 처리, 보존, 배포의 워크플로우를 상세히 설명합니다.

---

© 2025 SmileStory Inc. / WIA
弘益人間 (홍익인간) · Benefit All Humanity
