# 제5장: API 인터페이스 - 방언 데이터와의 대화

## 들어가며: 코드로 방언을 만나다

API (Application Programming Interface)는 방언 데이터와 애플리케이션을 연결하는 다리입니다. 이 장에서는 WIA-LANG-004가 제공하는 다양한 API 인터페이스를 살펴보고, 실제 사용 예제를 통해 방언 데이터를 프로그래밍 방식으로 활용하는 방법을 배웁니다.

## 5.1 API 설계 원칙

### 5.1.1 RESTful 아키텍처

WIA-LANG-004 API는 REST (Representational State Transfer) 원칙을 따릅니다:

| 원칙 | 설명 | 구현 방법 |
|------|------|-----------|
| **Client-Server** | 클라이언트와 서버의 분리 | HTTP 기반 통신 |
| **Stateless** | 상태 비저장 | 각 요청에 필요한 모든 정보 포함 |
| **Cacheable** | 캐시 가능 | Cache-Control 헤더 사용 |
| **Uniform Interface** | 일관된 인터페이스 | 표준 HTTP 메서드와 URI 구조 |
| **Layered System** | 계층화된 시스템 | 로드 밸런서, 캐시 서버 투명하게 사용 가능 |
| **Code on Demand** | 선택적 코드 전송 | JavaScript SDK 제공 |

### 5.1.2 HTTP 메서드 매핑

```yaml
http_methods:
  GET:
    purpose: "리소스 조회"
    examples:
      - "GET /dialects - 방언 목록 조회"
      - "GET /dialects/{id} - 특정 방언 상세 정보"
      - "GET /dialects/{id}/recordings - 방언의 녹음 목록"
    idempotent: true
    safe: true

  POST:
    purpose: "리소스 생성"
    examples:
      - "POST /dialects - 새 방언 등록"
      - "POST /recordings - 새 녹음 업로드"
      - "POST /annotations - 주석 추가"
    idempotent: false
    safe: false

  PUT:
    purpose: "리소스 전체 수정"
    examples:
      - "PUT /dialects/{id} - 방언 정보 전체 업데이트"
      - "PUT /speakers/{id} - 화자 정보 수정"
    idempotent: true
    safe: false

  PATCH:
    purpose: "리소스 부분 수정"
    examples:
      - "PATCH /dialects/{id} - 방언 정보 일부 업데이트"
      - "PATCH /transcriptions/{id} - 전사 일부 수정"
    idempotent: true
    safe: false

  DELETE:
    purpose: "리소스 삭제"
    examples:
      - "DELETE /dialects/{id} - 방언 삭제"
      - "DELETE /recordings/{id} - 녹음 삭제"
    idempotent: true
    safe: false
```

### 5.1.3 URI 구조

```
https://api.wia-lang.org/v1/{resource}/{id}/{sub-resource}

예시:
https://api.wia-lang.org/v1/dialects/ko-jeju
https://api.wia-lang.org/v1/dialects/ko-jeju/recordings
https://api.wia-lang.org/v1/dialects/ko-jeju/recordings/rec-001
https://api.wia-lang.org/v1/dialects/ko-jeju/recordings/rec-001/transcriptions
```

## 5.2 인증 및 권한 부여

### 5.2.1 OAuth 2.0 인증

```python
# OAuth 2.0 인증 흐름
import requests
from typing import Dict, Optional

class WIALangAuthenticator:
    """WIA-LANG-004 API 인증 클라이언트"""

    def __init__(self, client_id: str, client_secret: str,
                 auth_url: str = "https://auth.wia-lang.org"):
        self.client_id = client_id
        self.client_secret = client_secret
        self.auth_url = auth_url
        self.access_token: Optional[str] = None
        self.refresh_token: Optional[str] = None

    def authenticate(self, username: str, password: str) -> Dict:
        """사용자 이름과 비밀번호로 인증 (Resource Owner Password Credentials)"""
        token_endpoint = f"{self.auth_url}/oauth/token"

        payload = {
            "grant_type": "password",
            "client_id": self.client_id,
            "client_secret": self.client_secret,
            "username": username,
            "password": password,
            "scope": "read write"
        }

        response = requests.post(token_endpoint, data=payload)
        response.raise_for_status()

        token_data = response.json()
        self.access_token = token_data["access_token"]
        self.refresh_token = token_data.get("refresh_token")

        return {
            "access_token": self.access_token,
            "token_type": token_data["token_type"],
            "expires_in": token_data["expires_in"],
            "scope": token_data.get("scope", "")
        }

    def client_credentials(self) -> Dict:
        """클라이언트 인증 (Client Credentials Grant)"""
        token_endpoint = f"{self.auth_url}/oauth/token"

        payload = {
            "grant_type": "client_credentials",
            "client_id": self.client_id,
            "client_secret": self.client_secret,
            "scope": "read"
        }

        response = requests.post(token_endpoint, data=payload)
        response.raise_for_status()

        token_data = response.json()
        self.access_token = token_data["access_token"]

        return {
            "access_token": self.access_token,
            "token_type": token_data["token_type"],
            "expires_in": token_data["expires_in"]
        }

    def refresh_access_token(self) -> Dict:
        """액세스 토큰 갱신"""
        if not self.refresh_token:
            raise ValueError("Refresh token not available")

        token_endpoint = f"{self.auth_url}/oauth/token"

        payload = {
            "grant_type": "refresh_token",
            "client_id": self.client_id,
            "client_secret": self.client_secret,
            "refresh_token": self.refresh_token
        }

        response = requests.post(token_endpoint, data=payload)
        response.raise_for_status()

        token_data = response.json()
        self.access_token = token_data["access_token"]
        self.refresh_token = token_data.get("refresh_token", self.refresh_token)

        return {
            "access_token": self.access_token,
            "token_type": token_data["token_type"],
            "expires_in": token_data["expires_in"]
        }

    def get_auth_header(self) -> Dict[str, str]:
        """인증 헤더 생성"""
        if not self.access_token:
            raise ValueError("Not authenticated. Call authenticate() first.")

        return {
            "Authorization": f"Bearer {self.access_token}"
        }

# 사용 예시
auth = WIALangAuthenticator(
    client_id="your_client_id",
    client_secret="your_client_secret"
)

# 방법 1: 사용자 인증
token_info = auth.authenticate("username", "password")
print(f"Access Token: {token_info['access_token'][:20]}...")

# 방법 2: 클라이언트 인증 (서버 간 통신)
# token_info = auth.client_credentials()

# 인증 헤더 사용
headers = auth.get_auth_header()
print(f"Authorization Header: {headers['Authorization'][:30]}...")
```

### 5.2.2 권한 수준

```yaml
access_levels:
  public:
    description: "누구나 접근 가능"
    permissions:
      - "방언 목록 조회"
      - "공개 메타데이터 조회"
      - "샘플 데이터 다운로드 (5분 미만)"
    rate_limit: "100 requests/hour"

  registered:
    description: "등록된 사용자"
    permissions:
      - "public 권한 전부"
      - "전체 메타데이터 조회"
      - "녹음 파일 다운로드 (비상업적 용도)"
      - "주석 추가 (검토 후 반영)"
    rate_limit: "1000 requests/hour"

  researcher:
    description: "승인된 연구자"
    permissions:
      - "registered 권한 전부"
      - "대량 데이터 다운로드"
      - "API를 통한 주석 직접 추가"
      - "통계 및 분석 API"
    rate_limit: "5000 requests/hour"

  contributor:
    description: "데이터 제공자"
    permissions:
      - "researcher 권한 전부"
      - "새 방언 데이터 업로드"
      - "자신의 데이터 수정/삭제"
      - "품질 검증 도구 사용"
    rate_limit: "10000 requests/hour"

  admin:
    description: "시스템 관리자"
    permissions:
      - "모든 권한"
      - "사용자 관리"
      - "데이터 승인/거부"
      - "시스템 설정"
    rate_limit: "unlimited"
```

## 5.3 핵심 API 엔드포인트

### 5.3.1 방언 관리 API

```python
# WIA-LANG-004 API 클라이언트 (완전판)
import requests
from typing import Dict, List, Optional, BinaryIO
import json

class WIALangClient:
    """WIA-LANG-004 API 클라이언트"""

    def __init__(self, api_key: str, base_url: str = "https://api.wia-lang.org/v1"):
        self.api_key = api_key
        self.base_url = base_url
        self.session = requests.Session()
        self.session.headers.update({
            "Authorization": f"Bearer {api_key}",
            "Content-Type": "application/json",
            "Accept": "application/ld+json"
        })

    # ===== 방언 API =====

    def list_dialects(self, language: Optional[str] = None,
                     region: Optional[str] = None,
                     endangerment: Optional[str] = None,
                     page: int = 1, per_page: int = 20) -> Dict:
        """방언 목록 조회"""
        params = {
            "page": page,
            "per_page": per_page
        }

        if language:
            params["language"] = language
        if region:
            params["region"] = region
        if endangerment:
            params["endangerment"] = endangerment

        response = self.session.get(f"{self.base_url}/dialects", params=params)
        response.raise_for_status()
        return response.json()

    def get_dialect(self, dialect_id: str) -> Dict:
        """특정 방언 상세 정보"""
        response = self.session.get(f"{self.base_url}/dialects/{dialect_id}")
        response.raise_for_status()
        return response.json()

    def create_dialect(self, dialect_data: Dict) -> Dict:
        """새 방언 등록"""
        response = self.session.post(
            f"{self.base_url}/dialects",
            json=dialect_data
        )
        response.raise_for_status()
        return response.json()

    def update_dialect(self, dialect_id: str, updates: Dict) -> Dict:
        """방언 정보 수정"""
        response = self.session.patch(
            f"{self.base_url}/dialects/{dialect_id}",
            json=updates
        )
        response.raise_for_status()
        return response.json()

    def delete_dialect(self, dialect_id: str) -> bool:
        """방언 삭제"""
        response = self.session.delete(f"{self.base_url}/dialects/{dialect_id}")
        response.raise_for_status()
        return response.status_code == 204

    # ===== 녹음 API =====

    def list_recordings(self, dialect_id: str, page: int = 1, per_page: int = 20) -> Dict:
        """방언의 녹음 목록"""
        params = {"page": page, "per_page": per_page}
        response = self.session.get(
            f"{self.base_url}/dialects/{dialect_id}/recordings",
            params=params
        )
        response.raise_for_status()
        return response.json()

    def get_recording(self, recording_id: str) -> Dict:
        """특정 녹음 상세 정보"""
        response = self.session.get(f"{self.base_url}/recordings/{recording_id}")
        response.raise_for_status()
        return response.json()

    def upload_recording(self, dialect_id: str, audio_file: BinaryIO,
                        metadata: Dict) -> Dict:
        """녹음 파일 업로드"""
        # 먼저 메타데이터로 녹음 레코드 생성
        recording_data = {
            "dialect_id": dialect_id,
            **metadata
        }

        response = self.session.post(
            f"{self.base_url}/recordings",
            json=recording_data
        )
        response.raise_for_status()
        recording = response.json()

        # 파일 업로드
        upload_url = recording["upload_url"]
        files = {"file": audio_file}

        upload_response = requests.post(upload_url, files=files)
        upload_response.raise_for_status()

        return recording

    def download_recording(self, recording_id: str, format: str = "original") -> bytes:
        """녹음 파일 다운로드"""
        params = {"format": format}  # 'original', 'mp3', 'wav'
        response = self.session.get(
            f"{self.base_url}/recordings/{recording_id}/download",
            params=params
        )
        response.raise_for_status()
        return response.content

    # ===== 전사 API =====

    def get_transcription(self, recording_id: str) -> Dict:
        """녹음의 전사 조회"""
        response = self.session.get(
            f"{self.base_url}/recordings/{recording_id}/transcription"
        )
        response.raise_for_status()
        return response.json()

    def create_transcription(self, recording_id: str, transcription_data: Dict) -> Dict:
        """전사 생성/업데이트"""
        response = self.session.post(
            f"{self.base_url}/recordings/{recording_id}/transcription",
            json=transcription_data
        )
        response.raise_for_status()
        return response.json()

    def update_transcription(self, recording_id: str, utterance_id: str,
                           updates: Dict) -> Dict:
        """특정 발화 전사 수정"""
        response = self.session.patch(
            f"{self.base_url}/recordings/{recording_id}/transcription/utterances/{utterance_id}",
            json=updates
        )
        response.raise_for_status()
        return response.json()

    # ===== 주석 API =====

    def add_annotation(self, recording_id: str, utterance_id: str,
                      annotation: Dict) -> Dict:
        """주석 추가"""
        response = self.session.post(
            f"{self.base_url}/recordings/{recording_id}/utterances/{utterance_id}/annotations",
            json=annotation
        )
        response.raise_for_status()
        return response.json()

    def get_annotations(self, recording_id: str, utterance_id: str,
                       annotation_type: Optional[str] = None) -> List[Dict]:
        """주석 조회"""
        params = {}
        if annotation_type:
            params["type"] = annotation_type

        response = self.session.get(
            f"{self.base_url}/recordings/{recording_id}/utterances/{utterance_id}/annotations",
            params=params
        )
        response.raise_for_status()
        return response.json()

    # ===== 검색 API =====

    def search(self, query: str, filters: Optional[Dict] = None,
              facets: Optional[List[str]] = None, page: int = 1, per_page: int = 20) -> Dict:
        """통합 검색"""
        payload = {
            "query": query,
            "page": page,
            "per_page": per_page
        }

        if filters:
            payload["filters"] = filters
        if facets:
            payload["facets"] = facets

        response = self.session.post(
            f"{self.base_url}/search",
            json=payload
        )
        response.raise_for_status()
        return response.json()

    def advanced_search(self, criteria: Dict) -> Dict:
        """고급 검색"""
        response = self.session.post(
            f"{self.base_url}/search/advanced",
            json=criteria
        )
        response.raise_for_status()
        return response.json()

    # ===== 분석 API =====

    def compare_dialects(self, dialect_ids: List[str], features: List[str]) -> Dict:
        """방언 비교 분석"""
        payload = {
            "dialect_ids": dialect_ids,
            "features": features
        }

        response = self.session.post(
            f"{self.base_url}/analysis/compare",
            json=payload
        )
        response.raise_for_status()
        return response.json()

    def get_statistics(self, dialect_id: str, metrics: Optional[List[str]] = None) -> Dict:
        """방언 통계"""
        params = {}
        if metrics:
            params["metrics"] = ",".join(metrics)

        response = self.session.get(
            f"{self.base_url}/dialects/{dialect_id}/statistics",
            params=params
        )
        response.raise_for_status()
        return response.json()

    def phonetic_analysis(self, recording_id: str, utterance_id: str) -> Dict:
        """음성 분석"""
        response = self.session.get(
            f"{self.base_url}/recordings/{recording_id}/utterances/{utterance_id}/phonetic-analysis"
        )
        response.raise_for_status()
        return response.json()

# ===== 사용 예시 =====

# 클라이언트 초기화
client = WIALangClient(api_key="your_api_key_here")

# 1. 방언 목록 조회
print("=== 한국 방언 목록 ===")
korean_dialects = client.list_dialects(language="kor", per_page=10)
for dialect in korean_dialects["data"]:
    print(f"- {dialect['name']} ({dialect['id']})")

# 2. 제주 방언 상세 정보
print("\n=== 제주 방언 상세 ===")
jeju = client.get_dialect("ko-jeju")
print(f"이름: {jeju['name']}")
print(f"화자 수: {jeju['speakers']['estimatedCount']}")
print(f"위기 상태: {jeju['endangermentStatus']}")

# 3. 녹음 목록
print("\n=== 제주 방언 녹음 목록 ===")
recordings = client.list_recordings("ko-jeju", per_page=5)
for rec in recordings["data"]:
    print(f"- {rec['id']}: {rec['content']['genre']}, {rec['audioFile']['duration']}")

# 4. 검색
print("\n=== '혼저옵서예' 검색 ===")
search_results = client.search(
    query="혼저옵서예",
    filters={"dialect": "ko-jeju"},
    facets=["speaker_age", "recording_year"]
)
print(f"검색 결과: {search_results['total']}개")
for result in search_results["data"][:3]:
    print(f"- {result['snippet']}")

# 5. 방언 비교
print("\n=== 방언 비교: 제주 vs 경상 ===")
comparison = client.compare_dialects(
    dialect_ids=["ko-jeju", "ko-gyeongsang"],
    features=["vowel_system", "honorifics", "sentence_endings"]
)
for feature, data in comparison["features"].items():
    print(f"{feature}: 유사도 {data['similarity']:.2f}")

# 6. 통계
print("\n=== 제주 방언 통계 ===")
stats = client.get_statistics(
    "ko-jeju",
    metrics=["recording_count", "total_duration", "speaker_count", "vocabulary_size"]
)
for metric, value in stats.items():
    print(f"{metric}: {value}")
```

## 5.4 GraphQL API

REST API 외에도 GraphQL API를 제공하여 유연한 데이터 쿼리를 지원합니다.

### 5.4.1 GraphQL 스키마

```graphql
# WIA-LANG-004 GraphQL Schema

type Query {
  # 방언 조회
  dialect(id: ID!): Dialect
  dialects(
    language: String
    region: String
    endangerment: EndangermentStatus
    page: Int = 1
    perPage: Int = 20
  ): DialectConnection!

  # 녹음 조회
  recording(id: ID!): Recording
  recordings(
    dialectId: ID
    speakerId: ID
    genre: String
    page: Int = 1
    perPage: Int = 20
  ): RecordingConnection!

  # 검색
  search(
    query: String!
    filters: SearchFilters
    page: Int = 1
    perPage: Int = 20
  ): SearchResults!
}

type Mutation {
  # 방언 관리
  createDialect(input: DialectInput!): Dialect!
  updateDialect(id: ID!, input: DialectUpdateInput!): Dialect!
  deleteDialect(id: ID!): Boolean!

  # 녹음 관리
  createRecording(input: RecordingInput!): Recording!
  updateRecording(id: ID!, input: RecordingUpdateInput!): Recording!
  deleteRecording(id: ID!): Boolean!

  # 주석 추가
  addAnnotation(
    recordingId: ID!
    utteranceId: ID!
    annotation: AnnotationInput!
  ): Annotation!
}

type Dialect {
  id: ID!
  name: String!
  alternateNames: [String!]!
  iso639_3: String
  parentLanguage: String!
  endangermentStatus: EndangermentStatus!

  # 지리 정보
  geographic: GeographicInfo!

  # 화자 정보
  speakers: SpeakerInfo!

  # 언어학적 특징
  linguisticFeatures: LinguisticFeatures!

  # 관계
  recordings(page: Int = 1, perPage: Int = 20): RecordingConnection!
  relatedDialects: [Dialect!]!
}

type Recording {
  id: ID!
  dialect: Dialect!
  speaker: Speaker!
  recordedDate: DateTime!
  location: String!

  # 오디오 파일
  audioFile: AudioFile!

  # 내용
  content: RecordingContent!

  # 전사
  transcription: Transcription

  # 주석
  annotations: [Annotation!]!

  # 품질
  quality: QualityMetrics!
}

type Transcription {
  dialectOrthography: String!
  ipa: String
  standardKorean: String
  morphemes: [Morpheme!]!
  utterances: [Utterance!]!
}

type Utterance {
  id: ID!
  start: Float!
  end: Float!
  speaker: Speaker!
  text: String!
  ipa: String
  annotations: [Annotation!]!
}

enum EndangermentStatus {
  SAFE
  VULNERABLE
  DEFINITELY_ENDANGERED
  SEVERELY_ENDANGERED
  CRITICALLY_ENDANGERED
  EXTINCT
}

# 페이지네이션
type DialectConnection {
  edges: [DialectEdge!]!
  pageInfo: PageInfo!
  totalCount: Int!
}

type DialectEdge {
  node: Dialect!
  cursor: String!
}

type PageInfo {
  hasNextPage: Boolean!
  hasPreviousPage: Boolean!
  startCursor: String
  endCursor: String
}
```

### 5.4.2 GraphQL 쿼리 예시

```python
# GraphQL API 클라이언트
import requests
from typing import Dict, Any

class WIALangGraphQLClient:
    """WIA-LANG-004 GraphQL API 클라이언트"""

    def __init__(self, api_key: str, endpoint: str = "https://api.wia-lang.org/graphql"):
        self.api_key = api_key
        self.endpoint = endpoint
        self.headers = {
            "Authorization": f"Bearer {api_key}",
            "Content-Type": "application/json"
        }

    def query(self, query: str, variables: Dict[str, Any] = None) -> Dict:
        """GraphQL 쿼리 실행"""
        payload = {"query": query}
        if variables:
            payload["variables"] = variables

        response = requests.post(
            self.endpoint,
            json=payload,
            headers=self.headers
        )
        response.raise_for_status()

        result = response.json()
        if "errors" in result:
            raise Exception(f"GraphQL errors: {result['errors']}")

        return result["data"]

# 사용 예시
graphql_client = WIALangGraphQLClient(api_key="your_api_key")

# 예시 1: 방언 상세 정보 + 녹음 목록
query1 = """
query GetDialectWithRecordings($dialectId: ID!) {
  dialect(id: $dialectId) {
    id
    name
    endangermentStatus
    geographic {
      primaryRegion
      latitude
      longitude
    }
    speakers {
      estimatedCount
      ageDistribution {
        range
        count
      }
    }
    recordings(page: 1, perPage: 5) {
      edges {
        node {
          id
          recordedDate
          audioFile {
            duration
            format
          }
          speaker {
            age
            gender
          }
          quality {
            score
          }
        }
      }
      totalCount
    }
  }
}
"""

result1 = graphql_client.query(query1, {"dialectId": "ko-jeju"})
print("제주 방언 정보:")
print(f"- 이름: {result1['dialect']['name']}")
print(f"- 화자 수: {result1['dialect']['speakers']['estimatedCount']}")
print(f"- 녹음 수: {result1['dialect']['recordings']['totalCount']}")

# 예시 2: 검색 + 필터링
query2 = """
query SearchDialects($searchTerm: String!, $filters: SearchFilters) {
  search(query: $searchTerm, filters: $filters) {
    totalCount
    results {
      ... on Dialect {
        id
        name
        endangermentStatus
      }
      ... on Recording {
        id
        dialect {
          name
        }
        snippet
      }
    }
  }
}
"""

result2 = graphql_client.query(
    query2,
    {
        "searchTerm": "경상도",
        "filters": {
            "language": "kor",
            "endangerment": "VULNERABLE"
        }
    }
)
print(f"\n검색 결과: {result2['search']['totalCount']}개")

# 예시 3: 방언 비교 (복잡한 nested query)
query3 = """
query CompareDialects($dialectIds: [ID!]!) {
  dialects: nodes(ids: $dialectIds) {
    ... on Dialect {
      id
      name
      linguisticFeatures {
        phonetic
        lexical
        grammatical
      }
      recordings {
        totalCount
      }
    }
  }
}
"""

result3 = graphql_client.query(
    query3,
    {"dialectIds": ["ko-jeju", "ko-gyeongsang", "ko-jeolla"]}
)

print("\n방언 비교:")
for dialect in result3["dialects"]:
    print(f"\n{dialect['name']}:")
    print(f"  음운 특징: {len(dialect['linguisticFeatures']['phonetic'])}개")
    print(f"  어휘 특징: {len(dialect['linguisticFeatures']['lexical'])}개")
    print(f"  녹음 수: {dialect['recordings']['totalCount']}개")

# 예시 4: Mutation - 주석 추가
mutation = """
mutation AddPhoneticAnnotation(
  $recordingId: ID!
  $utteranceId: ID!
  $annotation: AnnotationInput!
) {
  addAnnotation(
    recordingId: $recordingId
    utteranceId: $utteranceId
    annotation: $annotation
  ) {
    id
    type
    content
    createdAt
    createdBy {
      name
    }
  }
}
"""

annotation_result = graphql_client.query(
    mutation,
    {
        "recordingId": "rec-001",
        "utteranceId": "utt-025",
        "annotation": {
            "type": "PHONETIC",
            "content": {
                "feature": "어두 경음화",
                "position": "혼저",
                "description": "표준어 '오-'가 제주어에서 'ㅎ'으로 시작"
            }
        }
    }
)

print(f"\n주석 추가됨: {annotation_result['addAnnotation']['id']}")
```

## 5.5 실시간 API (WebSocket)

실시간 업데이트가 필요한 경우 WebSocket API를 사용합니다.

```python
# WebSocket API 클라이언트
import asyncio
import websockets
import json
from typing import Callable, Dict

class WIALangWebSocketClient:
    """WIA-LANG-004 WebSocket API 클라이언트"""

    def __init__(self, api_key: str, ws_url: str = "wss://ws.wia-lang.org/v1"):
        self.api_key = api_key
        self.ws_url = ws_url
        self.websocket = None
        self.handlers: Dict[str, Callable] = {}

    async def connect(self):
        """WebSocket 연결"""
        uri = f"{self.ws_url}?token={self.api_key}"
        self.websocket = await websockets.connect(uri)
        print("WebSocket 연결됨")

        # 인증
        await self.websocket.send(json.dumps({
            "type": "auth",
            "token": self.api_key
        }))

        # 인증 응답 대기
        response = await self.websocket.recv()
        auth_result = json.loads(response)
        if auth_result.get("status") != "authenticated":
            raise Exception("인증 실패")

        print("인증 성공")

    async def subscribe(self, channel: str, filters: Dict = None):
        """채널 구독"""
        message = {
            "type": "subscribe",
            "channel": channel
        }
        if filters:
            message["filters"] = filters

        await self.websocket.send(json.dumps(message))
        print(f"채널 구독: {channel}")

    async def unsubscribe(self, channel: str):
        """채널 구독 해제"""
        await self.websocket.send(json.dumps({
            "type": "unsubscribe",
            "channel": channel
        }))
        print(f"구독 해제: {channel}")

    def on(self, event_type: str, handler: Callable):
        """이벤트 핸들러 등록"""
        self.handlers[event_type] = handler

    async def listen(self):
        """메시지 수신 대기"""
        try:
            async for message in self.websocket:
                data = json.loads(message)
                event_type = data.get("type")

                if event_type in self.handlers:
                    await self.handlers[event_type](data)
                else:
                    print(f"핸들러 없음: {event_type}")
                    print(data)

        except websockets.exceptions.ConnectionClosed:
            print("연결 종료됨")

    async def close(self):
        """연결 종료"""
        if self.websocket:
            await self.websocket.close()
            print("연결 닫힘")

# 사용 예시
async def main():
    client = WIALangWebSocketClient(api_key="your_api_key")

    # 이벤트 핸들러 정의
    async def on_new_recording(data):
        print(f"\n새 녹음 업로드됨:")
        print(f"  방언: {data['payload']['dialect_name']}")
        print(f"  시간: {data['payload']['duration']}초")
        print(f"  화자: {data['payload']['speaker_age']}세")

    async def on_annotation_added(data):
        print(f"\n새 주석 추가됨:")
        print(f"  유형: {data['payload']['annotation_type']}")
        print(f"  내용: {data['payload']['content'][:50]}...")

    async def on_transcription_completed(data):
        print(f"\n전사 완료:")
        print(f"  녹음 ID: {data['payload']['recording_id']}")
        print(f"  발화 수: {data['payload']['utterance_count']}")

    # 핸들러 등록
    client.on("new_recording", on_new_recording)
    client.on("annotation_added", on_annotation_added)
    client.on("transcription_completed", on_transcription_completed)

    # 연결 및 구독
    await client.connect()

    # 제주 방언 채널 구독
    await client.subscribe("dialect:ko-jeju")

    # 모든 한국 방언의 새 녹음 구독
    await client.subscribe("recordings:new", filters={"language": "kor"})

    # 메시지 수신 (Ctrl+C로 종료)
    try:
        await client.listen()
    except KeyboardInterrupt:
        print("\n종료 중...")
    finally:
        await client.close()

# 실행
# asyncio.run(main())
```

## 5.6 SDK 및 라이브러리

### 5.6.1 Python SDK

```python
# 공식 Python SDK 예시
from wia_lang import WIALang

# 초기화
wia = WIALang(api_key="your_api_key")

# 간편한 메서드
dialects = wia.dialects.list(language="kor")
jeju = wia.dialects.get("ko-jeju")

# 녹음 업로드 (파일 처리 자동화)
with open("recording.flac", "rb") as f:
    recording = wia.recordings.upload(
        dialect_id="ko-jeju",
        audio_file=f,
        metadata={
            "speaker_age": 85,
            "location": "제주시 구좌읍",
            "genre": "narrative"
        }
    )

# 전사 (자동 청킹 및 재시도)
transcription = wia.transcriptions.create(
    recording_id=recording.id,
    method="manual",  # or "automatic"
    language="ko-jeju"
)

# 검색 (Pythonic 인터페이스)
results = wia.search("혼저옵서예", dialect="ko-jeju")
for result in results:
    print(result.snippet)
```

### 5.6.2 JavaScript SDK

```javascript
// 공식 JavaScript SDK
import { WIALang } from '@wia-lang/sdk';

// 초기화
const wia = new WIALang({ apiKey: 'your_api_key' });

// Promise 기반
const dialects = await wia.dialects.list({ language: 'kor' });
const jeju = await wia.dialects.get('ko-jeju');

// 녹음 업로드
const fileInput = document.querySelector('#audio-file');
const recording = await wia.recordings.upload('ko-jeju', fileInput.files[0], {
  speaker_age: 85,
  location: '제주시 구좌읍',
  genre: 'narrative'
});

// 실시간 업데이트 구독
wia.subscribe('dialect:ko-jeju', (event) => {
  if (event.type === 'new_recording') {
    console.log('새 녹음:', event.data);
  }
});

// 검색
const results = await wia.search('혼저옵서예', {
  dialect: 'ko-jeju',
  page: 1,
  perPage: 20
});
```

## 5.7 에러 처리

```python
# 에러 처리 예시
from wia_lang import WIALang, WIALangError, AuthenticationError, NotFoundError, RateLimitError

wia = WIALang(api_key="your_api_key")

try:
    dialect = wia.dialects.get("invalid-id")

except AuthenticationError as e:
    print(f"인증 오류: {e.message}")
    print("API 키를 확인하세요")

except NotFoundError as e:
    print(f"리소스 없음: {e.resource_type} {e.resource_id}")
    print("ID를 확인하세요")

except RateLimitError as e:
    print(f"요청 한도 초과: {e.limit} requests/{e.window}")
    print(f"재시도: {e.retry_after}초 후")
    import time
    time.sleep(e.retry_after)
    # 재시도...

except WIALangError as e:
    print(f"API 오류: {e.message}")
    print(f"상태 코드: {e.status_code}")
    print(f"오류 코드: {e.error_code}")

except Exception as e:
    print(f"예기치 않은 오류: {e}")
```

## 5.8 마치며: API는 문화의 창구

API는 단순한 기술 인터페이스가 아닙니다. 그것은 방언이라는 소중한 문화유산을 세상과 연결하는 창구입니다.

**핵심 포인트:**
- REST, GraphQL, WebSocket 다양한 방식 지원
- OAuth 2.0 표준 인증
- Python, JavaScript SDK 제공
- 명확한 에러 처리

다음 장에서는 데이터 교환과 동기화 프로토콜을 살펴보겠습니다.

---

© 2025 SmileStory Inc. / WIA
弘益人間 (홍익인간) · Benefit All Humanity

**WIA-LANG-004 Dialect Preservation Standard v1.0**
