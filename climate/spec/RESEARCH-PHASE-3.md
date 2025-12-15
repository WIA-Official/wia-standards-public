# Phase 3 사전 조사 결과
# Phase 3 Research Findings: Communication Protocols

---

**작성일**: 2025년 12월 14일
**작성**: Claude Code (Opus 4.5)
**목적**: Climate 센서/시스템 간 통신 프로토콜 표준화 방향 도출

---

## 목차 (Table of Contents)

1. [WebSocket Protocol](#1-websocket-protocol)
2. [MQTT Protocol](#2-mqtt-protocol)
3. [OGC SensorThings API](#3-ogc-sensorthings-api)
4. [CMIP6/ESGF Data Access](#4-cmip6esgf-data-access)
5. [WMO BUFR/GRIB](#5-wmo-bufrgrib)
6. [기타 IoT 프로토콜](#6-기타-iot-프로토콜)
7. [프로토콜 비교 분석](#7-프로토콜-비교-분석)
8. [결론 및 설계 방향](#8-결론-및-설계-방향)

---

## 1. WebSocket Protocol

### 1.1 기술 개요

WebSocket은 RFC 6455로 표준화된 양방향 통신 프로토콜입니다. 단일 TCP 연결을 통해 클라이언트와 서버 간 실시간 양방향 데이터 교환이 가능합니다.

**핵심 특징**:
- **Full-duplex**: 양방향 동시 통신
- **Low overhead**: HTTP 대비 헤더 오버헤드 최소화
- **Persistent connection**: 연결 유지로 재연결 비용 제거
- **HTTP compatible**: 80/443 포트 사용, 기존 인프라 호환

### 1.2 기술 사양

| 항목 | 사양 |
|------|------|
| 표준 | RFC 6455 (2011) |
| 포트 | 80 (ws://), 443 (wss://) |
| 핸드셰이크 | HTTP Upgrade 메커니즘 |
| 프레이밍 | 2-14 바이트 헤더 |
| 최대 페이로드 | 2^63 바이트 (이론상) |
| 서브프로토콜 | 지원 (Sec-WebSocket-Protocol) |

### 1.3 연결 수립 과정

```
Client                                  Server
   |                                       |
   |-- HTTP Upgrade Request -------------->|
   |   GET /ws HTTP/1.1                    |
   |   Upgrade: websocket                  |
   |   Connection: Upgrade                 |
   |   Sec-WebSocket-Key: ...              |
   |                                       |
   |<-- HTTP 101 Switching Protocols ------|
   |    Upgrade: websocket                 |
   |    Sec-WebSocket-Accept: ...          |
   |                                       |
   |<========= WebSocket Connection ======>|
   |                                       |
```

### 1.4 Climate 도메인 적용 사례

- **실시간 센서 스트리밍**: DAC 플랜트 모니터링
- **기후 모델 시뮬레이션**: 실시간 결과 전송
- **대시보드 업데이트**: 환경 데이터 시각화
- **양방향 명령/제어**: 센서 설정 원격 변경

### 1.5 장단점

| 장점 | 단점 |
|------|------|
| 실시간 양방향 통신 | 상태 유지 필요 (stateful) |
| 낮은 지연 시간 | 프록시/방화벽 이슈 가능 |
| HTTP 대비 낮은 오버헤드 | 연결 유지 리소스 필요 |
| 브라우저 네이티브 지원 | 스케일링 복잡도 증가 |

---

## 2. MQTT Protocol

### 2.1 기술 개요

MQTT (Message Queuing Telemetry Transport)는 OASIS 표준으로 정의된 경량 publish-subscribe 메시징 프로토콜입니다. IoT와 저대역폭 환경에 최적화되어 있습니다.

**핵심 특징**:
- **Publish/Subscribe 패턴**: 송신자와 수신자 분리
- **경량 프로토콜**: 최소 2바이트 헤더
- **QoS 레벨**: 전송 신뢰성 보장
- **Broker 기반**: 메시지 라우팅 및 버퍼링

### 2.2 기술 사양

| 항목 | 사양 |
|------|------|
| 표준 | MQTT 5.0 (OASIS Standard) |
| 포트 | 1883 (mqtt), 8883 (mqtts) |
| 헤더 크기 | 2 바이트 (최소) |
| 최대 페이로드 | 256 MB |
| QoS 레벨 | 0, 1, 2 |
| 채택률 | 56% (IoT 프로토콜 중 1위, 2024) |

### 2.3 QoS (Quality of Service) 레벨

| QoS | 이름 | 설명 | 적용 사례 |
|-----|------|------|----------|
| 0 | At most once | 전송 보장 없음, 최소 오버헤드 | 실시간 센서 데이터 |
| 1 | At least once | 최소 1회 전달 보장, 중복 가능 | 환경 알림 |
| 2 | Exactly once | 정확히 1회 전달 보장 | 명령 제어 |

### 2.4 Climate 도메인 적용 사례

```
토픽 구조 예시:
  wia/climate/{device_id}/carbon_capture/data
  wia/climate/{device_id}/weather/status
  wia/climate/{facility_id}/vertical_farming/environment
  wia/climate/alerts/{severity}
```

**실제 구현 사례**:
- **기상 관측소**: DHT11/DHT22, BMP280 센서 데이터 전송
- **농업 IoT**: 토양 수분, 작물 건강 모니터링
- **대기질 모니터링**: MQ135 가스 센서 데이터

### 2.5 장단점

| 장점 | 단점 |
|------|------|
| 극도로 경량 (HTTP 대비 80% 적은 대역폭) | Broker 단일 장애점 |
| 비연결 상태에서 메시지 버퍼링 | 복잡한 라우팅 제한 |
| 다양한 QoS 옵션 | 토픽 기반만 지원 |
| TLS/SSL, OAuth 보안 지원 | 대용량 페이로드 비효율 |

---

## 3. OGC SensorThings API

### 3.1 기술 개요

OGC SensorThings API는 Open Geospatial Consortium에서 개발한 IoT 센서 데이터 표준입니다. 지리공간 기반의 통합 IoT 데이터 관리를 제공합니다.

**핵심 특징**:
- **OGC 표준**: 지리공간 데이터 호환
- **RESTful API**: HTTP 기반 접근
- **MQTT 지원**: 실시간 데이터 스트리밍
- **INSPIRE 호환**: EU 환경 데이터 인프라 지원

### 3.2 표준 구조

| Part | 이름 | 설명 | 발표년도 |
|------|------|------|---------|
| Part 1 | Sensing | IoT 센서 관측 데이터 관리 | 2016 |
| Part 2 | Tasking Core | 센서/액추에이터 제어 | 2019 |

### 3.3 데이터 모델

```
                    ┌─────────────────┐
                    │      Thing      │
                    │ (Environmental  │
                    │    Station)     │
                    └────────┬────────┘
                             │
              ┌──────────────┼──────────────┐
              ▼              ▼              ▼
       ┌──────────┐   ┌──────────┐   ┌──────────┐
       │ Location │   │ Datastream│   │  Sensor  │
       └──────────┘   └─────┬────┘   └──────────┘
                            │
                            ▼
                    ┌──────────────┐
                    │ Observation  │
                    │ (Measured    │
                    │   Values)    │
                    └──────────────┘
```

### 3.4 Climate 도메인 적용 사례

**EU 프로젝트**:
- **USAGE** (Urban Data Space for Green Deal): 도시 환경/기후 데이터
- **AD4GD** (All Data for Green Deal): 기후 변화 완화 전략

**INSPIRE 구현**:
- 수질 모니터링 (프랑스, 독일)
- 실시간 대기질 (유럽 전역)
- 스마트 시티 (함부르크)

### 3.5 장단점

| 장점 | 단점 |
|------|------|
| 지리공간 데이터 네이티브 지원 | 복잡한 데이터 모델 |
| REST + MQTT 이중 지원 | 구현 복잡도 높음 |
| EU INSPIRE 호환 | 소규모 프로젝트에 과도 |
| 자체 기술 메타데이터 | 학습 곡선 존재 |

---

## 4. CMIP6/ESGF Data Access

### 4.1 기술 개요

ESGF (Earth System Grid Federation)는 기후 모델 데이터 배포를 위한 분산 아카이브 시스템입니다. CMIP6 데이터를 전 세계에 제공합니다.

**핵심 특징**:
- **분산 노드**: 전 세계 모델링 센터에 분산 저장
- **OPeNDAP**: HTTP 기반 원격 데이터 접근
- **Globus**: 대용량 데이터 전송
- **무제한 접근**: 계정 없이 데이터 접근 가능

### 4.2 데이터 접근 포털

| 노드 | 위치 | URL |
|------|------|-----|
| PCMDI/LLNL | 미국 (캘리포니아) | aims2.llnl.gov |
| IPSL | 프랑스 | esgf-node.ipsl.upmc.fr |
| DKRZ | 독일 | esgf-data.dkrz.de |
| CEDA | 영국 | esgf-index1.ceda.ac.uk |

### 4.3 접근 방식

| 방식 | 설명 | 용도 |
|------|------|------|
| **Web UI** | 브라우저 기반 검색/다운로드 | 탐색, 소량 데이터 |
| **Wget Script** | 자동 생성 다운로드 스크립트 | 배치 다운로드 |
| **pyesgf** | Python API | 프로그래밍 접근 |
| **OPeNDAP** | HTTP 원격 접근 | 부분 데이터 접근 |
| **Globus** | 고성능 전송 | 대용량 데이터 |

### 4.4 Climate 도메인 적용

WIA Climate의 Phase 1에서 정의한 `climate_model` 데이터 타입은 CMIP6와 호환되도록 설계되었습니다:

```json
{
  "model": {
    "source_id": "CESM2",
    "institution_id": "NCAR",
    "experiment_id": "ssp245"
  },
  "variable": {
    "name": "tas",
    "units": "K"
  },
  "scenario": {
    "ssp": "ssp245"
  }
}
```

---

## 5. WMO BUFR/GRIB

### 5.1 기술 개요

WMO (World Meteorological Organization)에서 정의한 기상 데이터 교환 형식입니다.

**두 가지 형식**:
- **BUFR** (Binary Universal Form for the Representation): 관측 데이터
- **GRIB** (GRIdded Binary): 격자 데이터 (예보, 분석)

### 5.2 BUFR 특징

| 항목 | 설명 |
|------|------|
| 형식 | 바이너리, 테이블 기반 |
| 용도 | 기상 관측 데이터 |
| 특징 | 자기 기술적 (self-describing) |
| 인코딩 | WMO 테이블 참조 |
| 전송 | GTS (Global Telecommunication System) |

### 5.3 GRIB 특징

| 버전 | 설명 | 상태 |
|------|------|------|
| GRIB1 | 1985년 개발, 밀리도 해상도 | 사용 중단 권고 |
| GRIB2 | 2003년 개발, 마이크로도 해상도 | 현재 표준 |

### 5.4 관련 소프트웨어

- **ecCodes** (ECMWF): C, Fortran, Python API
- **wgrib2**: GRIB2 파일 처리
- **NetCDF**: 과학 데이터 형식 (상호 변환 가능)

### 5.5 WIA Climate 연동

WIA Climate는 JSON 기반이므로 BUFR/GRIB과 직접 호환되지 않으나, 변환 레이어를 통해 연동 가능:

```
BUFR/GRIB → ecCodes → JSON → WIA Climate Message
```

---

## 6. 기타 IoT 프로토콜

### 6.1 CoAP (Constrained Application Protocol)

| 항목 | 설명 |
|------|------|
| 전송 | UDP 기반 |
| 패턴 | RESTful (GET, POST, PUT, DELETE) |
| 용도 | 저전력 센서, 제약된 네트워크 |
| 장점 | HTTP 대비 경량, NAT 투과 |
| 단점 | 신뢰성 낮음 (UDP) |

### 6.2 AMQP (Advanced Message Queuing Protocol)

| 항목 | 설명 |
|------|------|
| 전송 | TCP 기반 |
| 패턴 | 메시지 큐잉 |
| 용도 | 엔터프라이즈 메시징 |
| 장점 | 높은 신뢰성, 트랜잭션 지원 |
| 단점 | 복잡도, 오버헤드 |

### 6.3 시리얼 통신

| 프로토콜 | 설명 | 속도 |
|---------|------|------|
| UART | 비동기 직렬 | 115200 bps |
| SPI | 동기 직렬, 다중 장치 | ~50 Mbps |
| I2C | 2-wire, 다중 장치 | ~3.4 Mbps |
| USB | 범용 직렬 버스 | 1.5 Mbps ~ 20 Gbps |

---

## 7. 프로토콜 비교 분석

### 7.1 주요 프로토콜 비교

| 특성 | WebSocket | MQTT | HTTP/REST | CoAP |
|------|-----------|------|-----------|------|
| **전송** | TCP | TCP | TCP | UDP |
| **패턴** | Bidirectional | Pub/Sub | Request/Response | Request/Response |
| **오버헤드** | 낮음 | 매우 낮음 | 높음 | 매우 낮음 |
| **실시간** | 우수 | 우수 | 보통 | 우수 |
| **신뢰성** | 높음 | QoS 가변 | 높음 | 낮음 |
| **브라우저** | 지원 | 미지원 | 지원 | 미지원 |
| **확장성** | 보통 | 우수 | 우수 | 보통 |

### 7.2 Climate 도메인 적합성

| 용도 | 권장 프로토콜 | 이유 |
|------|-------------|------|
| **센서 스트리밍** | MQTT QoS 0 | 경량, 실시간, 손실 허용 |
| **대시보드** | WebSocket | 양방향, 브라우저 지원 |
| **명령/제어** | MQTT QoS 2 | 정확한 전달 보장 |
| **데이터 조회** | HTTP/REST | 표준 API, 캐싱 |
| **대용량 전송** | HTTP + Chunked | 안정성 |
| **로컬 센서** | Serial (I2C/SPI) | 저지연, 직접 연결 |

### 7.3 WIA Climate 멀티-트랜스포트 전략

```
┌─────────────────────────────────────────────────────────────┐
│                    WIA Climate Protocol                      │
│                   (Application Layer)                        │
├─────────────────────────────────────────────────────────────┤
│                    Message Format (JSON)                     │
│    { protocol, version, messageId, type, payload }          │
├──────────────┬──────────────┬──────────────┬────────────────┤
│  WebSocket   │    MQTT      │  HTTP/REST   │    Serial      │
│  Transport   │  Transport   │  Transport   │   Transport    │
├──────────────┴──────────────┴──────────────┴────────────────┤
│                     Network Layer                            │
│                   (TCP/UDP/Serial)                           │
└─────────────────────────────────────────────────────────────┘
```

---

## 8. 결론 및 설계 방향

### 8.1 핵심 설계 원칙

1. **트랜스포트 독립성**: 메시지 형식은 전송 계층과 분리
2. **Phase 1/2 호환**: 기존 데이터 형식을 페이로드로 그대로 사용
3. **다중 프로토콜**: WebSocket, MQTT, HTTP 모두 지원
4. **경량 설계**: IoT 센서 환경 고려
5. **확장 가능**: 새로운 전송 방식 추가 용이

### 8.2 메시지 형식 설계

```json
{
  "protocol": "wia-climate",
  "version": "1.0.0",
  "messageId": "550e8400-e29b-41d4-a716-446655440000",
  "timestamp": 1702483200000,
  "type": "data",
  "payload": {
    "/* Phase 1 ClimateMessage 형식 */"
  }
}
```

### 8.3 메시지 유형

| Type | 방향 | 설명 |
|------|-----|------|
| `connect` | C → S | 연결 요청 |
| `connect_ack` | S → C | 연결 응답 |
| `disconnect` | Both | 연결 종료 |
| `data` | S → C | 센서 데이터 전송 |
| `command` | C → S | 명령 전송 |
| `command_ack` | S → C | 명령 응답 |
| `subscribe` | C → S | 데이터 구독 |
| `unsubscribe` | C → S | 구독 해제 |
| `error` | Both | 에러 메시지 |
| `ping` | C → S | 연결 확인 |
| `pong` | S → C | 연결 확인 응답 |

### 8.4 구현 우선순위

| 우선순위 | 전송 방식 | 이유 |
|---------|----------|------|
| **1** | WebSocket | 실시간 양방향, 브라우저 지원 |
| **2** | MQTT | IoT 표준, 널리 사용 |
| **3** | HTTP/REST | 조회/설정 API |
| **4** | Serial | 로컬 센서 연결 |

### 8.5 보안 고려사항

- **TLS/SSL**: 모든 전송에서 암호화 지원
- **인증**: JWT 또는 API 키 기반
- **권한**: 토픽/채널별 접근 제어
- **메시지 무결성**: 서명 옵션

### 8.6 다음 단계

1. PHASE-3-PROTOCOL.md 상세 스펙 작성
2. Protocol JSON Schema 정의
3. Rust protocol 모듈 구현
4. WebSocket transport 구현
5. 테스트 및 예제 작성

---

## 참고 자료

### WebSocket
- [RFC 6455 - The WebSocket Protocol](https://tools.ietf.org/html/rfc6455)
- [WebSocket - Wikipedia](https://en.wikipedia.org/wiki/WebSocket)

### MQTT
- [MQTT.org - The Standard for IoT Messaging](https://mqtt.org/)
- [HiveMQ MQTT Essentials](https://www.hivemq.com/mqtt/)
- [EMQX MQTT Guide](https://www.emqx.com/en/blog/the-easiest-guide-to-getting-started-with-mqtt)

### OGC SensorThings API
- [OGC SensorThings API](https://ogcapi.ogc.org/sensorthings/)
- [OGC SensorThings for European Green Deal](https://www.ogc.org/blog-article/ogc-sensorthings-api-for-european-green-deal-data-spaces/)

### CMIP6/ESGF
- [CMIP Data Access](https://wcrp-cmip.org/cmip-data-access/)
- [CMIP6 Guidance for Data Users](https://pcmdi.llnl.gov/CMIP6/Guide/dataUsers.html)

### WMO Standards
- [WMO Data Access and Use](https://community.wmo.int/en/activity-areas/wmo-space-programme-wsp/satellite-data-formats-standards)
- [GRIB - Wikipedia](https://en.wikipedia.org/wiki/GRIB)

---

弘益人間 - Benefit All Humanity 🌍
