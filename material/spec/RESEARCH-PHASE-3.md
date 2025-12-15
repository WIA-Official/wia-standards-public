# Phase 3 사전 조사 결과
# Phase 3 Research Findings

---

**작성일**: 2025년 12월 14일
**작성**: Claude Code (Opus 4.5)
**목적**: Material Science 데이터 통신 프로토콜 조사 및 표준 설계 방향 도출

---

## 목차 (Table of Contents)

1. [데이터 교환 프로토콜 비교](#1-데이터-교환-프로토콜-비교)
2. [기존 재료과학 API 조사](#2-기존-재료과학-api-조사)
3. [실시간 데이터 스트리밍](#3-실시간-데이터-스트리밍)
4. [데이터 형식 및 직렬화](#4-데이터-형식-및-직렬화)
5. [인증 및 보안](#5-인증-및-보안)
6. [결론 및 설계 방향](#6-결론-및-설계-방향)

---

## 1. 데이터 교환 프로토콜 비교

### 1.1 REST API

**특징**:
- HTTP 기반의 표준 아키텍처
- 리소스 중심의 URI 설계
- Stateless 통신
- 캐싱 지원

**장점**:
- 광범위한 언어/플랫폼 지원
- 브라우저에서 직접 호출 가능
- 디버깅 및 테스트 용이
- 기존 웹 인프라 활용 가능

**단점**:
- Over-fetching/Under-fetching 문제
- 실시간 통신에 부적합
- 대용량 데이터 전송 시 오버헤드

**Material Science 적용**:
- 재료 데이터베이스 쿼리에 적합
- OPTIMADE, Materials Project에서 채택
- Phase 1/2의 JSON Schema와 자연스러운 연동

### 1.2 GraphQL

**특징**:
- 쿼리 언어 기반 API
- 클라이언트가 필요한 데이터만 요청
- 단일 엔드포인트

**장점**:
- 유연한 데이터 요청
- Over-fetching 방지
- 강력한 타입 시스템

**단점**:
- 복잡한 학습 곡선
- 캐싱 어려움
- 서버 측 구현 복잡

**Material Science 적용**:
- 복잡한 물성 쿼리에 유용
- 다중 재료 비교 조회에 적합
- 현재 주요 재료 DB에서는 미채택

### 1.3 gRPC

**특징**:
- Protocol Buffers 기반 바이너리 직렬화
- HTTP/2 기반 양방향 스트리밍
- 코드 자동 생성

**장점**:
- 고성능 (REST 대비 4배+ 빠름)
- 강력한 타입 정의
- 양방향 스트리밍
- 무손실 바이너리 전송

**단점**:
- 브라우저 직접 지원 불가
- 디버깅 어려움
- 학습 곡선

**Material Science 적용**:
- 대용량 시뮬레이션 데이터 전송
- 마이크로서비스 간 통신
- 측정 장비 실시간 데이터 스트리밍

### 1.4 WebSocket

**특징**:
- 전이중(Full-duplex) 양방향 통신
- 단일 TCP 연결 유지
- 낮은 지연 시간

**장점**:
- 실시간 데이터 스트리밍
- 낮은 오버헤드
- 브라우저 지원

**단점**:
- Stateful 연결 관리 필요
- 로드 밸런싱 복잡
- 재연결 로직 필요

**Material Science 적용**:
- 측정 장비 실시간 모니터링
- 시뮬레이션 진행 상황 스트리밍
- 협업 데이터 편집

---

## 2. 기존 재료과학 API 조사

### 2.1 OPTIMADE API

**URL**: https://www.optimade.org/

**개요**:
- Open Databases Integration for Materials Design
- 재료 데이터베이스 간 상호운용성 표준
- 60M+ 구조, 30+ 데이터베이스 연합

**기술 스택**:
- RESTful API
- JSON:API 스펙 준수
- Semantic Versioning

**버전**: v1.2.0 (2024년 최신)

**URL 형식**:
```
https://{provider}/v1/structures?filter=elements HAS "Fe"
https://{provider}/v1/structures/{id}
```

**응답 형식**:
```json
{
  "data": {
    "type": "structures",
    "id": "example/1",
    "attributes": {
      "chemical_formula_reduced": "Fe2O3",
      "nelements": 2,
      "elements": ["Fe", "O"],
      "lattice_vectors": [[...], [...], [...]]
    }
  },
  "links": {
    "self": "https://provider/v1/structures/example/1"
  },
  "meta": {
    "api_version": "1.2.0",
    "query": {...}
  }
}
```

**WIA Material 관련성**:
- 구조 호환성 참고
- 필터 문법 참고
- JSON:API 패턴 채택 검토

### 2.2 Materials Project API

**URL**: https://api.materialsproject.org/

**개요**:
- Lawrence Berkeley National Laboratory 운영
- 15만+ 무기 화합물 데이터
- Python SDK (pymatgen) 제공

**기술 스택**:
- RESTful API (v2)
- JSON 형식
- API 키 인증 (HTTPS 필수)

**URL 형식**:
```
https://api.materialsproject.org/materials/{material_id}
https://api.materialsproject.org/materials/summary?formula=Fe2O3
```

**인증**:
```http
GET /materials/mp-123 HTTP/1.1
Host: api.materialsproject.org
X-API-Key: your-api-key
```

**응답 형식**:
```json
{
  "material_id": "mp-541837",
  "formula": "Bi2Se3",
  "formation_energy_per_atom": -0.234,
  "band_gap": 0.3,
  "density": 7.68,
  "structure": {...}
}
```

### 2.3 NOMAD (Novel Materials Discovery)

**URL**: https://nomad-lab.eu/

**개요**:
- 유럽 연구 인프라
- 1억+ 시뮬레이션 데이터
- AFLOW, OQMD, Materials Project 원본 데이터 보유

**기술 스택**:
- RESTful API
- NeXus 형식 (HDF5 기반)
- OPTIMADE 호환

**데이터 형식**:
- JSON (메타데이터)
- HDF5 (대용량 배열 데이터)
- NeXus (실험 데이터)

**특징**:
- 계산 및 실험 데이터 통합
- AI 툴킷 제공
- FAIR 원칙 준수

### 2.4 AFLOW (Automatic FLOW)

**URL**: http://aflowlib.org/

**개요**:
- Duke University 운영
- 360만+ 화합물
- 자동화된 계산 워크플로우

**API 형식**:
```
http://aflowlib.org/API/aflux/?
  catalog(lib)
  filter(Egap>0.5)
  format(json)
```

**특징**:
- AFLUX 쿼리 언어
- 프로토타입 분류 체계
- 열역학 데이터 특화

---

## 3. 실시간 데이터 스트리밍

### 3.1 Lab Streaming Layer (LSL)

**URL**: https://labstreaminglayer.readthedocs.io/

**개요**:
- 신경생리학 연구용 데이터 스트리밍
- 150+ 장치 지원
- 네트워크 시간 동기화

**특징**:
- Zero-configuration
- 자동 시간 동기화
- LAN 기반

**Material Science 적용 가능성**:
- 다중 센서 동기화 참고
- 스트리밍 아키텍처 참고

### 3.2 MQTT over WebSocket

**개요**:
- IoT 데이터 수집 표준
- 경량 pub/sub 프로토콜
- QoS 레벨 지원

**장점**:
- 낮은 대역폭 사용
- 비동기 메시징
- 브라우저 지원

**적용 예시**:
- 환경 센서 데이터 수집
- 장비 상태 모니터링
- 실험 이벤트 알림

### 3.3 WebSocket for Instrumentation

**연구 사례**:
Koheron 등의 연구에서 WebSocket을 사용한 계측기 제어 웹 인터페이스 구현

**장점**:
- HTTP 대비 50% 지연 시간 개선
- 대역폭 효율성
- 브라우저 네이티브 지원

**적용**:
- 실시간 측정 데이터 시각화
- 원격 장비 제어
- 데이터 수집 모니터링

---

## 4. 데이터 형식 및 직렬화

### 4.1 JSON

**장점**:
- 인간 가독성
- 광범위한 지원
- 디버깅 용이

**단점**:
- 바이너리 데이터 비효율
- 파싱 오버헤드

**적용**:
- 메타데이터
- 구조/물성 데이터
- API 응답

### 4.2 Protocol Buffers (Protobuf)

**장점**:
- 컴팩트한 바이너리 형식
- 스키마 강제
- 빠른 직렬화/역직렬화

**단점**:
- 스키마 관리 필요
- 디버깅 어려움

**적용**:
- 대용량 배열 데이터
- 고빈도 스트리밍
- 마이크로서비스 통신

### 4.3 HDF5 / NeXus

**장점**:
- 대용량 과학 데이터에 최적화
- 계층적 구조
- 부분 읽기 지원

**단점**:
- 복잡한 구현
- 웹 친화적이지 않음

**적용**:
- 시뮬레이션 결과
- 실험 원시 데이터
- 이미지/스펙트럼 데이터

### 4.4 MessagePack

**장점**:
- JSON과 유사한 구조
- 바이너리로 컴팩트
- 빠른 처리

**적용**:
- 효율적인 JSON 대체
- 웹소켓 바이너리 모드

---

## 5. 인증 및 보안

### 5.1 API 키 인증

**Materials Project 방식**:
```http
X-API-Key: your-api-key
```

**장점**:
- 간단한 구현
- 사용량 추적 용이

**단점**:
- 키 유출 위험
- 세분화된 권한 어려움

### 5.2 OAuth 2.0 / JWT

**장점**:
- 표준화된 인증
- 세분화된 권한
- 토큰 만료 관리

**적용**:
- 사용자 인증
- 제3자 앱 연동
- 세션 관리

### 5.3 HTTPS 필수

모든 주요 재료 DB API는 HTTPS 필수:
- Materials Project: HTTP 요청 시 403 반환
- OPTIMADE: TLS 권장
- NOMAD: HTTPS 기본

---

## 6. 결론 및 설계 방향

### 6.1 프로토콜 선택

| 용도 | 권장 프로토콜 | 이유 |
|------|-------------|------|
| **데이터 조회** | REST API | OPTIMADE/MP 호환, 범용성 |
| **실시간 스트리밍** | WebSocket | 낮은 지연, 브라우저 지원 |
| **고성능 통신** | gRPC (옵션) | 대용량/고빈도 데이터 |
| **메시징** | MQTT (옵션) | IoT 센서 통합 |

### 6.2 메시지 형식 설계

**기본 구조** (Phase 1/2와 일관성 유지):

```json
{
  "protocol": "wia-material",
  "version": "1.0.0",
  "message_id": "uuid-v4",
  "timestamp": "2025-12-14T00:00:00Z",
  "type": "메시지 유형",
  "payload": {
    // Phase 1 MaterialData 또는 요청/응답 데이터
  }
}
```

### 6.3 메시지 유형

| Type | 방향 | 설명 |
|------|-----|------|
| `query` | Client → Server | 데이터 조회 요청 |
| `query_response` | Server → Client | 조회 응답 |
| `subscribe` | Client → Server | 스트리밍 구독 |
| `unsubscribe` | Client → Server | 구독 해제 |
| `data` | Server → Client | 데이터 스트림 (Phase 1 형식) |
| `command` | Client → Server | 명령 전송 |
| `command_ack` | Server → Client | 명령 응답 |
| `error` | Both | 에러 메시지 |
| `ping` | Client → Server | 연결 확인 |
| `pong` | Server → Client | 연결 확인 응답 |

### 6.4 OPTIMADE 호환성

WIA Material Protocol은 OPTIMADE와 상호 변환 가능하도록 설계:

```
OPTIMADE Request → WIA Protocol → WIA Response → OPTIMADE Response
```

### 6.5 구현 우선순위

1. **Phase 3 Core**: REST API + WebSocket (필수)
2. **Phase 3 Extended**: gRPC, MQTT (선택)
3. **Phase 4**: 다른 DB와 연동

### 6.6 보안 기본 원칙

1. HTTPS 필수
2. API 키 기본 인증
3. JWT 옵션 지원
4. Rate limiting
5. CORS 설정

---

## 참고 문헌 (References)

### 표준 및 스펙

- [OPTIMADE Specification](https://www.optimade.org/optimade)
- [JSON:API Specification](https://jsonapi.org/)
- [RFC 6455 - WebSocket Protocol](https://tools.ietf.org/html/rfc6455)
- [gRPC Documentation](https://grpc.io/docs/)

### 재료 데이터베이스

- [Materials Project API](https://api.materialsproject.org/docs)
- [NOMAD Repository](https://nomad-lab.eu/)
- [AFLOW](http://aflowlib.org/)

### 논문

- Andersen et al., "OPTIMADE, an API for exchanging materials data," Sci. Data 8, 217 (2021)
- Evans et al., "Developments and applications of the OPTIMADE API," Digital Discovery (2024)
- Ong et al., "The Materials Application Programming Interface (API)," Comput. Mater. Sci. (2015)
- Draxl & Scheffler, "NOMAD: The FAIR concept," MRS Bulletin (2018)

---

<div align="center">

**Phase 3 Research Complete**

---

弘益人間 🤟

</div>
